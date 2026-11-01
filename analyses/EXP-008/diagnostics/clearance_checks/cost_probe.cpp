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
 grid_map::GridMap ref({"distance_field","angle_field","speed_field"}),cost({"collision_layer"});ref.setGeometry(grid_map::Length(10.,10.),.1,grid_map::Position(2.,0.));cost.setGeometry(grid_map::Length(10.,10.),.05,grid_map::Position(3.,0.));
 for(grid_map::GridMapIterator it(ref);!it.isPastEnd();++it){grid_map::Position p;ref.getPosition(*it,p);ref.at("distance_field",*it)=std::abs(p.y());ref.at("angle_field",*it)=0.;ref.at("speed_field",*it)=1.;}
 for(grid_map::GridMapIterator it(cost);!it.isPastEnd();++it){grid_map::Position p;cost.getPosition(*it,p);cost.at("collision_layer",*it)=(p.x()>.6&&p.x()<.9&&p.y()>.1&&p.y()<.4)?100.:0.;}

 auto state=mppi::cpu::State::Zero().eval();state(3)=1.;
 mppi::cpu::ForwardMPPI solver(params.common,params.forward_mppi);solver.set_reference_map(ref);solver.set_obstacle_map(cost);
 auto controls=solver.get_control_seq();controls.setZero();
 auto [path0,c0,hit0,e0]=solver.get_predictive_seq(state,controls);
 cost.add("clearance_cost",0.5);solver.set_obstacle_map(cost);
 auto [path1,c1,hit1,e1]=solver.get_predictive_seq(state,controls);
 double expected=params.common.prediction_step_size*.5;
 std::cout<<"cost_delta="<<c1-c0<<" expected="<<expected<<" collision_delta="<<hit1-hit0<<"\n";
 return std::abs((c1-c0)-expected)<1e-5 && hit1==hit0 ? 0:1;
}
