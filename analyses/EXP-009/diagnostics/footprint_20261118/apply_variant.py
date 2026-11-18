from pathlib import Path
root=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031')
src=root/'ws_footprint/src/svg_mppi_ros2'
h=src/'include/mppi_controller/mpc_base.hpp'
s=h.read_text().replace('#include "mppi_controller/common.hpp"','#include "mppi_controller/common.hpp"\n#include "mppi_controller/oriented_footprint.hpp"')
s=s.replace('grid_map::GridMap obstacle_map_;','grid_map::GridMap obstacle_map_;\n        OrientedFootprint footprint_;')
h.write_text(s)
p=src/'src/mpc_base.cpp';s=p.read_text()
s='#include <grid_map_core/iterators/GridMapIterator.hpp>\n'+s
s=s.replace('void MPCBase::set_obstacle_map(const grid_map::GridMap& obstacle_map) { obstacle_map_ = obstacle_map; }','''void MPCBase::set_obstacle_map(const grid_map::GridMap& obstacle_map) {
        if (!obstacle_map.exists("raw_occupancy") || !obstacle_map["raw_occupancy"].allFinite())
            throw std::invalid_argument("footprint_v1 requires finite raw_occupancy");
        obstacle_map_ = obstacle_map;
        std::vector<OrientedFootprint::Point> occupied;
        for (grid_map::GridMapIterator it(obstacle_map_); !it.isPastEnd(); ++it) {
            if (obstacle_map_.at("raw_occupancy", *it) > 0.) {
                grid_map::Position p; obstacle_map_.getPosition(*it, p);
                occupied.push_back({p.x(),p.y()});
            }
        }
        const auto center=obstacle_map_.getPosition();
        const auto half=obstacle_map_.getLength()/2.;
        footprint_.reset(occupied, obstacle_map_.getResolution(), center.x()-half.x(), center.x()+half.x(),
                         center.y()-half.y(), center.y()+half.y());
    }''')
a=s.index('        // calc cost for each state',s.index('MPCBase::state_cost(const StateSeq& local_state_seq'))
b=s.index('\n    // calculate state cost',a)
s=s[:a]+'''        (void)obstacle_map;
        double collision=0., clearance=0.;
        for(size_t i=0;i<prediction_step_size_;++i) {
            const auto cost=footprint_.cost(local_state_seq(i,STATE_SPACE::x),local_state_seq(i,STATE_SPACE::y),local_state_seq(i,STATE_SPACE::yaw));
            collision+=cost.collision*collision_weight_;
            clearance+=cost.clearance;
        }
        return {collision+clearance,collision};
    }
''' +s[b:]
a=s.index('        const auto clearance_cost =');b=s.index('        for (size_t i =',a)
s=s[:a]+'''        (void)local_obstacle_map;
        const auto body_cost = [&](const State& state) {
            return footprint_.cost(state(STATE_SPACE::x), state(STATE_SPACE::y), state(STATE_SPACE::yaw));
        };
'''+s[b:]
s=s.replace('sum_ref_cost += clearance_cost(local_state);','const auto footprint_cost = body_cost(local_state);\n            sum_ref_cost += footprint_cost.clearance;')
a=s.index('            // collision cost',s.index('const auto body_cost'));b=s.index('        }\n',s.index('sum_collision_cost += collision_cost * collision_weight_;',a))
s=s[:a]+'''            sum_collision_cost += footprint_cost.collision * collision_weight_;
'''+s[b:]
a=s.index('        sum_ref_cost += clearance_cost(local_terminal_state);');b=s.index('        return std::make_pair(sum_ref_cost',a)
s=s[:a]+'''        const auto terminal_cost=body_cost(local_terminal_state);
        sum_ref_cost += terminal_cost.clearance;
        sum_collision_cost += terminal_cost.collision * collision_weight_;

'''+s[b:]
p.write_text(s)
p=src/'scripts/sim_inputs.py';s=p.read_text().replace("{'collision_layer':inflated.astype(np.float32)*100,","{'raw_occupancy':occupied.astype(np.float32)*100,'collision_layer':inflated.astype(np.float32)*100,");p.write_text(s)
p=src/'src/mppi_sim_node.cpp';s=p.read_text().replace('costmap_.exists("collision_layer")','costmap_.exists("raw_occupancy") && costmap_["raw_occupancy"].allFinite() && costmap_.exists("collision_layer")');p.write_text(s)
p=root/'tools/manual_session.py';s=p.read_text().replace("'consistent_v1','svg_repro_v1']","'consistent_v1','svg_repro_v1','footprint_v1']").replace("'svg_repro_v1':'ws_svg_repro'","'svg_repro_v1':'ws_svg_repro','footprint_v1':'ws_footprint'");p.write_text(s)
