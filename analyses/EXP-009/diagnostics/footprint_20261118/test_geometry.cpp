#include "mppi_controller/oriented_footprint.hpp"
#include <iomanip>
#include <iostream>
int main(){double x,y,yaw,ox,oy;std::cout<<std::setprecision(17);while(std::cin>>x>>y>>yaw>>ox>>oy){mppi::cpu::OrientedFootprint f; f.reset({{ox,oy}},.05,-5,5,-5,5);const auto a=f.cost(x,y,yaw);std::cout<<a.collision<<' '<<a.clearance<<'\n';}}
