#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <iostream>
#include <numeric>
#include "mppi_controller/forward_guard.hpp"
using namespace mppi::cpu;
void require(bool value,const char* what){if(!value) throw std::runtime_error(what);}
int main(){
 grid_map::GridMap ref({"angle_field"});ref.setGeometry(grid_map::Length(10.,10.),.05,grid_map::Position(2.,0.));ref["angle_field"].setZero();
 StateSeq straight=StateSeq::Zero(45,mppi::STATE_SPACE::dim);
 for(int i=0;i<45;++i){straight(i,0)=.075*i;straight(i,3)=1.5;}
 require(check_forward(straight,ref).valid,"straight failed");
 require(std::abs(check_forward(straight,ref).progress-3.3)<1e-10,"progress incorrect");
 auto reverse=straight;reverse.col(2).setConstant(3.141592653589793);require(!check_forward(reverse,ref).valid,"reverse heading accepted");
 auto backwards=straight;backwards.col(0)*=-.2;require(!check_forward(backwards,ref).valid,"backward translation accepted");
 auto stopped=straight;stopped.setZero();require(check_forward(stopped,ref).valid,"stationary state rejected");
 auto nan=straight;nan(2,2)=NAN;require(!check_forward(nan,ref).valid,"NaN accepted");
 auto outside=straight;outside(44,0)=20.;require(!check_forward(outside,ref).valid,"outside accepted");
 OrientedFootprint clear,wall;clear.reset({},.05,-2,8,-5,5);wall.reset({{1.,0.}},.05,-2,8,-5,5);
 require(std::string(validate_forward_path(straight,ref,clear,0,0,0))=="ok","valid final rejected");
 require(std::string(validate_forward_path(straight,ref,wall,0,0,0))!="ok","wall final accepted");
 StateSeq sparse=StateSeq::Zero(2,mppi::STATE_SPACE::dim);sparse(1,0)=1.;sparse.col(3).setConstant(1.5);
 OrientedFootprint middle;middle.reset({{.5,0.}},.05,-2,8,-5,5);
 require(middle.cost(0,0,0).collision==0 && middle.cost(1,0,0).collision==0,"sparse endpoints not free");
 require(std::string(validate_forward_path(sparse,ref,middle,0,0,0))!="ok","intermediate collision missed");
 auto weights=forward_weights({1000.,-500.,1001.},{1.,invalid_forward_cost,2.},3.);
 require(weights[1]==0. && weights[0]>0 && weights[2]>0 && std::abs(std::accumulate(weights.begin(),weights.end(),0.)-1.)<1e-12,"mask weights failed");
 bool threw=false;try{forward_weights({1.,2.},{invalid_forward_cost,invalid_forward_cost},3.);}catch(const std::runtime_error&){threw=true;}require(threw,"all invalid did not stop");
 // Correct forward travel round a curve must remain admissible.
 grid_map::GridMap curved=ref;
 for(grid_map::GridMapIterator it(curved);!it.isPastEnd();++it){grid_map::Position p;curved.getPosition(*it,p);curved.at("angle_field",*it)=std::atan2(p.x(),2.-p.y());}
 StateSeq arc=straight;
 for(int i=0;i<45;++i){double a=i*.025;arc(i,0)=2*std::sin(a);arc(i,1)=2-2*std::cos(a);arc(i,2)=a;}
 require(check_forward(arc,curved).valid,"forward curve rejected");
 std::cout<<"{\"passed\":true,\"checks\":13,\"includes\":[\"forward curve\",\"reverse heading\",\"backward translation\",\"nonfinite\",\"interpolated collision\",\"masked weights\",\"all invalid stop\"]}\n";
}
