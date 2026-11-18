from pathlib import Path
r=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');src=r/'ws_forward_guard/src/svg_mppi_ros2'
p=src/'src/mpc_base.cpp';s=p.read_text();s='#include "mppi_controller/forward_guard.hpp"\n'+s
old='return std::make_pair(sum_ref_cost + sum_collision_cost, sum_collision_cost);'
assert s.count(old)==1
s=s.replace(old,'''const auto forward=check_forward(global_state_seq,ref_path_map);
        // Exact exclusion is applied in both algorithms' final importance weights.
        if(!forward.valid || sum_collision_cost>0.)
            return {invalid_forward_cost+std::abs(sum_ref_cost)+sum_collision_cost,sum_collision_cost};
        // Signed projection on the local reference tangent; not nearest-index reward.
        sum_ref_cost -= 2.0 * forward.progress;
        return std::make_pair(sum_ref_cost + sum_collision_cost, sum_collision_cost);''');p.write_text(s)
for name in ['forward_mppi.cpp','stein_variational_guided_mppi.cpp']:
 p=src/'src'/name;s=p.read_text();s='#include "mppi_controller/forward_guard.hpp"\n'+s
 old='return softmax(costs_with_control_term, lambda_, thread_num_);';assert s.count(old)==1
 s=s.replace(old,'return forward_weights(costs_with_control_term, prior_samples_with_costs.costs_, lambda_);')
 old='        prev_control_seq_ = updated_control_seq;';assert s.count(old)==1
 s=s.replace(old,'''        // The nonlinear rollout of an average need not inherit sample feasibility.
        const auto [mean_path,mean_cost,mean_collision]=mpc_base_ptr_->get_predictive_seq(initial_state,updated_control_seq);
        if(!std::isfinite(mean_cost) || mean_cost>=invalid_forward_cost || mean_collision>0.) {
            const size_t best=std::distance(weights.begin(),std::max_element(weights.begin(),weights.end()));
            updated_control_seq=prior_samples_ptr_->noised_control_seq_samples_.at(best);
        }
        prev_control_seq_ = updated_control_seq;''')
 p.write_text(s)
p=src/'src/mppi_sim_node.cpp';s=p.read_text();s='#include <grid_map_core/iterators/GridMapIterator.hpp>\n#include "mppi_controller/forward_guard.hpp"\n'+s
old='enabled_=msg->data; if(!enabled_) publish_drive(0.,0.);';assert old in s
s=s.replace(old,'enabled_=msg->data && !guard_latched_; if(!enabled_) publish_drive(0.,last_steer_);')
s=s.replace('    const auto begin=Clock::now();','    const auto begin=Clock::now();\n    if(guard_latched_){publish_drive(0.,last_steer_);return;}')
s=s.replace('      solver_->set_obstacle_map(costmap_);','''      // Snapshot is immutable throughout solve and validation (single executor).
      std::vector<mppi::cpu::OrientedFootprint::Point> occupied;
      for(grid_map::GridMapIterator it(costmap_);!it.isPastEnd();++it) {
        if(costmap_.at("raw_occupancy",*it)>0.) {
          grid_map::Position p;costmap_.getPosition(*it,p);occupied.push_back({p.x(),p.y()});
        }
      }
      const auto center=costmap_.getPosition();const auto half=costmap_.getLength()/2.;
      footprint_.reset(occupied,costmap_.getResolution(),center.x()-half.x(),center.x()+half.x(),center.y()-half.y(),center.y()+half.y());
      solver_->set_obstacle_map(costmap_);''')
old='      last_steer_=std::clamp(control(0,0),min_steer_,max_steer_);publish_drive(speed_,last_steer_);';assert old in s;s=s.replace(old,'')
old='      metric(calc_,solve_ms);metric(collision_,rate);metric(cycle_,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());status("running");'
assert old in s;s=s.replace(old,'''      const char* verdict=mppi::cpu::validate_forward_path(prediction,refmap_,footprint_,state_(0),state_(1),state_(2));
      if(std::string(verdict)!="ok") throw std::runtime_error(verdict);
      if(!std::isfinite(ref_cost) || ref_cost>=mppi::cpu::invalid_forward_cost || collision_cost>0.)
        throw std::runtime_error("returned_path_invalid_cost");
      last_steer_=std::clamp(control(0,0),min_steer_,max_steer_);publish_drive(speed_,last_steer_);
      metric(calc_,solve_ms);metric(collision_,rate);metric(cycle_,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());status("running");''')
old='} catch(const std::exception& e){enabled_=false;publish_drive(0.,0.);status(std::string("solver_error: ")+e.what());}'
assert old in s;s=s.replace(old,'''} catch(const std::exception& e){
      guard_latched_=true;enabled_=false;publish_drive(0.,last_steer_);
      metric(cycle_,std::chrono::duration<double,std::milli>(Clock::now()-begin).count());
      status(std::string("guard_stop: ")+e.what());
    }''')
s=s.replace('bool odom_ok_=false,','mppi::cpu::OrientedFootprint footprint_;\n  bool guard_latched_=false;\n  bool odom_ok_=false,');p.write_text(s)
p=r/'tools/manual_session.py';s=p.read_text().replace("'svg_repro_v1','footprint_v1']","'svg_repro_v1','footprint_v1','forward_guard_v1']").replace("'footprint_v1':'ws_footprint'","'footprint_v1':'ws_footprint','forward_guard_v1':'ws_forward_guard'");p.write_text(s)
