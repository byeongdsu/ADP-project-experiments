#include <grid_map_core/iterators/GridMapIterator.hpp>
#include "mppi_controller/forward_guard.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "mppi_controller/forward_mppi.hpp"
#include "mppi_controller/stein_variational_guided_mppi.hpp"
using Clock=std::chrono::steady_clock;
template<class T> T read(const YAML::Node& root,const std::string& path,const T& fallback) {
  auto node=YAML::Clone(root); std::istringstream stream(path); std::string part;
  while(std::getline(stream,part,'/')) { if(!node[part]) return fallback; node=node[part]; }
  return node.as<T>();
}
class MPPISim:public rclcpp::Node {
 public:
  MPPISim():Node("mppi_sim_controller") {
    if(!declare_parameter("simulation_only",true)) throw std::runtime_error("Only simulation is supported");
    auto cfg=YAML::LoadFile(declare_parameter<std::string>("config_file",""));
    mode_=declare_parameter<std::string>("mpc_mode",read<std::string>(cfg,"mpc_mode","forward_mppi"));
    mppi::Params params{};
#include "load_params.inc"
    params.common.is_localize_less_mode=false;
    params.common.steer_1st_delay=read<double>(cfg,"steer_1st_delay",0.1);
    speed_=params.common.reference_speed; period_=read<double>(cfg,"control_sampling_time",.025);
    steer_tau_=std::max(params.common.steer_1st_delay,params.common.prediction_interval);
    min_steer_=params.common.min_steer_angle; max_steer_=params.common.max_steer_angle;
    if(speed_<=0 || speed_>3.) throw std::runtime_error("Initial simulation adapter supports simulation speed (0,3] m/s");
    if(mode_=="forward_mppi") solver_=std::make_unique<mppi::cpu::ForwardMPPI>(params.common,params.forward_mppi);
    else if(mode_=="svg_mppi") solver_=std::make_unique<mppi::cpu::SVGuidedMPPI>(params.common,params.svg_mppi);
    else throw std::runtime_error("Unsupported mode");
    state_=mppi::cpu::State::Zero();
    drive_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/mppi_sim/drive",10);
    path_=create_publisher<nav_msgs::msg::Path>("/mppi/best_path",10);
    calc_=create_publisher<std_msgs::msg::Float32>("/mppi/calculation_time",10);
    cycle_=create_publisher<std_msgs::msg::Float32>("/mppi/cycle_time",10);
    collision_=create_publisher<std_msgs::msg::Float32>("/mppi/collision_rate",10);
    status_=create_publisher<std_msgs::msg::String>("/mppi/status",rclcpp::QoS(1).transient_local());
    odom_=create_subscription<nav_msgs::msg::Odometry>("/car_state/odom_GT",rclcpp::QoS(1).best_effort(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      if(msg->header.frame_id!="map" || msg->child_frame_id!="car_state/base_link") return;
      const auto& p=msg->pose.pose; const auto& q=p.orientation;
      state_(0)=p.position.x; state_(1)=p.position.y;
      state_(2)=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      state_(3)=msg->twist.twist.linear.x; odom_time_=Clock::now(); odom_ok_=state_.allFinite();
    });
    cost_=create_subscription<grid_map_msgs::msg::GridMap>("/mppi/local_costmap",rclcpp::QoS(1).best_effort(),[this](grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {
      cost_ok_=false;
      if(msg->header.frame_id!="car_state/base_link") return;
      cost_ok_=grid_map::GridMapRosConverter::fromMessage(*msg,costmap_) && costmap_.exists("raw_occupancy") && costmap_["raw_occupancy"].allFinite() && costmap_.exists("collision_layer") && costmap_["collision_layer"].allFinite() && costmap_.exists("clearance_cost") && costmap_["clearance_cost"].allFinite();
      cost_time_=Clock::now();
    });
    reference_=create_subscription<grid_map_msgs::msg::GridMap>("/mppi/reference_sdf",rclcpp::QoS(1).transient_local(),[this](grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {
      ref_ok_=false;
      if(msg->header.frame_id!="map") return;
      ref_ok_=grid_map::GridMapRosConverter::fromMessage(*msg,refmap_);
      for(const auto& layer:{"distance_field","angle_field","speed_field"}) ref_ok_=ref_ok_ && refmap_.exists(layer) && refmap_[layer].allFinite();
      if(ref_ok_) solver_->set_reference_map(refmap_);
    });
    enable_=create_subscription<std_msgs::msg::Bool>("/mppi/enable",10,[this](std_msgs::msg::Bool::ConstSharedPtr msg){enabled_=msg->data && !guard_latched_; if(!enabled_) publish_drive(0.,last_steer_);});
    timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{tick();});
    RCLCPP_INFO(get_logger(),"Simulation-only %s, GT pose from odom, %.2f m/s; waiting for /mppi/enable",mode_.c_str(),speed_);
  }
  void stop(){publish_drive(0.,0.);}
 private:
  void publish_drive(double speed,double steer) {
    ackermann_msgs::msg::AckermannDriveStamped cmd;cmd.header.stamp=now();cmd.header.frame_id="car_state/base_link";
    cmd.drive.speed=speed;cmd.drive.steering_angle=steer;drive_->publish(cmd);
  }
  void metric(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub,double value){std_msgs::msg::Float32 msg;msg.data=value;pub->publish(msg);}
  void status(const std::string& text) {if(text!=last_status_){std_msgs::msg::String msg;msg.data=text;status_->publish(msg);RCLCPP_INFO(get_logger(),"%s",text.c_str());last_status_=text;}}
  void tick() {
    const auto begin=Clock::now();
    if(guard_latched_){publish_drive(0.,last_steer_);return;}
    const bool fresh=odom_ok_ && cost_ok_ && std::chrono::duration<double>(begin-odom_time_).count()<.5 && std::chrono::duration<double>(begin-cost_time_).count()<.5;
    if(!fresh || !ref_ok_){status("waiting_or_stale_inputs");publish_drive(0.,0.);return;}
    if(!refmap_.isInside(grid_map::Position(state_(0),state_(1)))) {status("outside_reference_map");publish_drive(0.,0.);return;}
    if(!validated_) {
      const grid_map::Position here(state_(0),state_(1));
      const double distance=refmap_.atPosition("distance_field",here);
      const double heading=refmap_.atPosition("angle_field",here);
      const double angle_error=std::atan2(std::sin(state_(2)-heading),std::cos(state_(2)-heading));
      const double ref_speed=refmap_.atPosition("speed_field",here);
      const double self_cost=costmap_.atPosition("collision_layer",grid_map::Position(0.,0.));
      if(distance>.2 || std::abs(angle_error)>.3 || std::abs(ref_speed-speed_)>.01 || self_cost>0.) {
        status("initial_grid_alignment_failed");publish_drive(0.,0.);return;
      }
      RCLCPP_INFO(get_logger(),"Input alignment checked: distance=%.4f m heading_error=%.4f rad ref_speed=%.2f self_cost=%.1f",distance,angle_error,ref_speed,self_cost);
      validated_=true;
    }
    if(!enabled_){status("ready_disabled");publish_drive(0.,0.);return;}
    const double dt=last_tick_.time_since_epoch().count()?std::clamp(std::chrono::duration<double>(begin-last_tick_).count(),0.,.1):period_;
    last_tick_=begin; state_(4)+=(last_steer_-state_(4))*(1-std::exp(-dt/steer_tau_));
    try {
      // Snapshot is immutable throughout solve and validation (single executor).
      std::vector<mppi::cpu::OrientedFootprint::Point> occupied;
      for(grid_map::GridMapIterator it(costmap_);!it.isPastEnd();++it) {
        if(costmap_.at("raw_occupancy",*it)>0.) {
          grid_map::Position p;costmap_.getPosition(*it,p);occupied.push_back({p.x(),p.y()});
        }
      }
      const auto center=costmap_.getPosition();const auto half=costmap_.getLength()/2.;
      footprint_.reset(occupied,costmap_.getResolution(),center.x()-half.x(),center.x()+half.x(),center.y()-half.y(),center.y()+half.y());
      solver_->set_obstacle_map(costmap_);
      const auto solve_begin=Clock::now();
      auto [control,rate]=solver_->solve(state_);
      const double solve_ms=std::chrono::duration<double,std::milli>(Clock::now()-solve_begin).count();
      if(!control.allFinite() || !std::isfinite(rate)) throw std::runtime_error("non-finite solver output");

      auto [prediction,ref_cost,collision_cost,input_error]=solver_->get_predictive_seq(state_,control);
      nav_msgs::msg::Path path;path.header.stamp=now();path.header.frame_id="map";
      for(int i=0;i<prediction.rows();++i){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=prediction(i,0);p.pose.position.y=prediction(i,1);p.pose.orientation.z=std::sin(prediction(i,2)*.5);p.pose.orientation.w=std::cos(prediction(i,2)*.5);path.poses.push_back(p);}path_->publish(path);
      const char* verdict=mppi::cpu::validate_forward_path(prediction,refmap_,footprint_,state_(0),state_(1),state_(2));
      if(std::string(verdict)!="ok") throw std::runtime_error(verdict);
      if(!std::isfinite(ref_cost) || ref_cost>=mppi::cpu::invalid_forward_cost || collision_cost>0.)
        throw std::runtime_error("returned_path_invalid_cost");
      last_steer_=std::clamp(control(0,0),min_steer_,max_steer_);publish_drive(speed_,last_steer_);
      metric(calc_,solve_ms);metric(collision_,rate);metric(cycle_,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());status("running");
    } catch(const std::exception& e){
      guard_latched_=true;enabled_=false;publish_drive(0.,last_steer_);
      metric(cycle_,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());
      status(std::string("guard_stop: ")+e.what());
    }
  }
  std::string mode_,last_status_; double speed_,period_,steer_tau_,min_steer_,max_steer_,last_steer_=0.;
  mppi::cpu::OrientedFootprint footprint_;
  bool guard_latched_=false;
  bool odom_ok_=false,cost_ok_=false,ref_ok_=false,enabled_=false,validated_=false;
  Clock::time_point odom_time_,cost_time_,last_tick_;
  mppi::cpu::State state_; grid_map::GridMap costmap_,refmap_;
  std::unique_ptr<mppi::cpu::MPCTemplate> solver_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr calc_,cycle_,collision_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr cost_,reference_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<MPPISim>();rclcpp::spin(node);node->stop();}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
