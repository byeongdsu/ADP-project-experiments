#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <grid_map_core/GridMap.hpp>
#include "mppi_controller/common.hpp"
#include "mppi_controller/oriented_footprint.hpp"
namespace mppi { namespace cpu {
constexpr double invalid_forward_cost=1e9;
struct ForwardCheck { bool valid; double progress; const char* reason; };
inline ForwardCheck check_forward(const StateSeq& path,const grid_map::GridMap& reference) {
    if(path.rows()<2 || !path.allFinite()) return {false,0.,"nonfinite_or_empty_path"};
    double progress=0.;
    for(int i=0;i<path.rows();++i) {
        const grid_map::Position p(path(i,STATE_SPACE::x),path(i,STATE_SPACE::y));
        if(!reference.isInside(p)) return {false,progress,"outside_reference"};
        const double angle=reference.atPosition("angle_field",p);
        if(!std::isfinite(angle) || std::cos(path(i,STATE_SPACE::yaw)-angle)<=0.)
            return {false,progress,"opposite_heading"};
        if(i+1<path.rows()) {
            const double step=(path(i+1,STATE_SPACE::x)-p.x())*std::cos(angle)
                             +(path(i+1,STATE_SPACE::y)-p.y())*std::sin(angle);
            if(step < -1e-3) return {false,progress,"backward_reference_step"};
            progress+=step;
        }
    }
    if(path.col(STATE_SPACE::vel).cwiseAbs().maxCoeff()>.05 && progress<=1e-4)
        return {false,progress,"no_forward_progress"};
    return {true,progress,"ok"};
}
// Use original rollout cost for feasibility, and augmented cost only for weighting.
inline std::vector<double> forward_weights(const std::vector<double>& augmented,
                                           const std::vector<double>& rollout,double lambda) {
    if(augmented.size()!=rollout.size() || lambda<=0.) throw std::runtime_error("invalid_forward_weights");
    double minimum=INFINITY;
    for(size_t i=0;i<rollout.size();++i)
        if(std::isfinite(rollout[i]) && rollout[i]<invalid_forward_cost && std::isfinite(augmented[i]))
            minimum=std::min(minimum,augmented[i]);
    if(!std::isfinite(minimum)) throw std::runtime_error("no_collision_free_forward_candidate");
    std::vector<double> weights(rollout.size(),0.);double sum=0.;
    for(size_t i=0;i<rollout.size();++i) {
        if(std::isfinite(rollout[i]) && rollout[i]<invalid_forward_cost && std::isfinite(augmented[i]))
            weights[i]=std::exp(-(augmented[i]-minimum)/lambda);
        sum+=weights[i];
    }
    for(auto& w:weights) w/=sum;
    return weights;
}
inline const char* validate_forward_path(const StateSeq& path,const grid_map::GridMap& reference,
                                         const OrientedFootprint& footprint,double ego_x,double ego_y,double ego_yaw) {
    const auto direction=check_forward(path,reference);
    if(!direction.valid) return direction.reason;
    const double c=std::cos(ego_yaw),s=std::sin(ego_yaw);
    // Validate the returned mean/fallback, not just its constituent samples.
    // Interpolate at <=.02m or <=2deg between predicted states.
    for(int i=0;i+1<path.rows();++i) {
        const double dyaw=std::atan2(std::sin(path(i+1,2)-path(i,2)),std::cos(path(i+1,2)-path(i,2)));
        const int n=std::max(1,static_cast<int>(std::ceil(std::max(
            std::hypot(path(i+1,0)-path(i,0),path(i+1,1)-path(i,1))/.02,std::abs(dyaw)/(.03490658503988659)))));
        for(int j=0;j<=n;++j) {
            const double f=static_cast<double>(j)/n;
            const double x=path(i,0)+f*(path(i+1,0)-path(i,0));
            const double y=path(i,1)+f*(path(i+1,1)-path(i,1));
            const double yaw=path(i,2)+f*dyaw;
            const grid_map::Position pos(x,y);
            if(!reference.isInside(pos)) return "outside_reference";
            if(std::cos(yaw-reference.atPosition("angle_field",pos))<=0.) return "opposite_heading";
            const double dx=x-ego_x,dy=y-ego_y;
            if(footprint.cost(c*dx+s*dy,-s*dx+c*dy,yaw-ego_yaw).collision>0.) return "returned_path_collision_or_grid_exit";
        }
    }
    return "ok";
}
}}
