// Offline fixed-input ForwardMPPI probe. No ROS nodes or simulation commands.
#include <omp.h>
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include "mppi_controller/forward_mppi.hpp"
template<class T>T read(const YAML::Node& root,const std::string& path,const T& fallback){auto node=YAML::Clone(root);std::istringstream stream(path);std::string part;while(std::getline(stream,part,'/')){if(!node[part])return fallback;node=node[part];}return node.as<T>();}
int main(int argc,char** argv){
 if(argc!=2)return 2;omp_set_dynamic(0);const auto cfg=YAML::LoadFile(argv[1]);mppi::Params params{};
#include "load_params.inc"
 params.common.is_localize_less_mode=false;params.common.steer_1st_delay=cfg["steer_1st_delay"].as<double>();

 grid_map::GridMap ref({"distance_field","angle_field","speed_field"}),cost({"collision_layer"});
 ref.setGeometry(grid_map::Length(20.,10.),.1,grid_map::Position(3.,0.));
 cost.setGeometry(grid_map::Length(16.,8.),.005,grid_map::Position(3.,0.));
 ref["distance_field"].setZero();ref["angle_field"].setZero();ref["speed_field"].setConstant(3.);cost["collision_layer"].setZero();
 auto state=mppi::cpu::State::Zero().eval();state(3)=3.;state(4)=.3;
 mppi::cpu::ForwardMPPI solver(params.common,params.forward_mppi);solver.set_reference_map(ref);solver.set_obstacle_map(cost);
 auto controls=solver.get_control_seq();controls.setZero();auto [pred,rc,cc,err]=solver.get_predictive_seq(state,controls);
 grid_map::Position first(pred(1,0),pred(1,1));cost.atPosition("collision_layer",first)=100.;solver.set_obstacle_map(cost);
 auto [pred2,rc2,cc2,err2]=solver.get_predictive_seq(state,controls);
 std::cout<<"{\"initial_steer_rad\":0.3,\"first_global_prediction_xy\":["<<first.x()<<","<<first.y()<<"],\"cost_at_published_first_global_point\":"<<cost.atPosition("collision_layer",first)<<",\"solver_reported_collision_cost\":"<<cc2<<",\"note\":\"Synthetic coordinate-consistency check, not a replay of an actual collision\"}\n";
 return cost.atPosition("collision_layer",first)==100. && cc2==0.?0:1;
}
