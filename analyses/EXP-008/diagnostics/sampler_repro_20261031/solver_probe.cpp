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
 auto state=mppi::cpu::State::Zero().eval();state(1)=.2;state(2)=.05;state(3)=1.;int mismatch=0,nonfinite=0;double maxdiff=0.,first_steer=0.;
 for(int trial=0;trial<3;++trial){mppi::cpu::ForwardMPPI a(params.common,params.forward_mppi),b(params.common,params.forward_mppi);a.set_reference_map(ref);b.set_reference_map(ref);a.set_obstacle_map(cost);b.set_obstacle_map(cost);
  for(int call=0;call<5;++call){auto [aa,ar]=a.solve(state);auto [bb,br]=b.solve(state);double diff=(aa-bb).cwiseAbs().maxCoeff();maxdiff=std::max(maxdiff,diff);if(diff!=0.||ar!=br)++mismatch;if(!aa.allFinite()||!bb.allFinite())++nonfinite;if(trial==0&&call==0)first_steer=aa(0,0);}
 }
 std::cout<<"{\"fixture\":\"fixed state, straight reference and a local rectangular cost obstacle\",\"threads\":"<<params.common.thread_num<<",\"pairs\":15,\"mismatched_pairs\":"<<mismatch<<",\"max_control_difference\":"<<maxdiff<<",\"nonfinite_pairs\":"<<nonfinite<<",\"first_steering\":"<<first_steer<<"}\n";
 return mismatch||nonfinite?1:0;
}
