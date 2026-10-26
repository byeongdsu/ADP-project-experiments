> 중단·원복: 사용자 요청으로 EXP-005 고정 배치·진단 기능을 제거했습니다. 아래 내용은 과거 준비 기록이며 실행하지 마세요.

# EXP-005/R01 실행 명령

각터미널에서 race. 기존실행종료1→2→3→4, 시작1→3→4→2. 고정출발점RViz수동변경없음.

터미널1:
```bash
ros2 launch stack_master base_system_launch.xml map_name:=hyhall12 pose_source:=pf
```

터미널3:
```bash
ros2 launch race_manager race_manager_launch.xml map_name:=hyhall12 seed:=2026 num_obstacles:=2 plot:=true fixed_layout_file:=/home/misys/ADP-project-experiments/runs/EXP-005/R01/fixed_layout.json log_dir:=/home/misys/ADP-project-experiments/runs/EXP-005/R01
```

터미널4:
```bash
ros2 bag record -o /home/misys/ADP-project-experiments/runs/EXP-005/R01/telemetry   /drive /car_state/odom_GT /car_state/odom /car_state/frenet/odom   /global_waypoints /global_waypoints_scaled /local_waypoints   /sim/collision /sim/obstacles /sim/obstacles/markers   /pf/viz/inferred_pose /car_state/pose_GT /scan   /state /perception/obstacles /trailing/gap_data   /planner/avoidance/otwpnts   /state_machine/active_avoidance_wpnts /state_machine/diagnostics
```

터미널2:
```bash
python3 /home/misys/ADP-project-experiments/scripts/check_pf_start.py --output /home/misys/ADP-project-experiments/runs/EXP-005/R01/start_check.json && ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP
```

PF검사실패시PP는자동시작되지않는다. 수동으로PP만시작하지말고검사결과를확인한다. 3랩완료·첫충돌·15초연속정지중하나에서종료1→2→3→4. 매랩장애물배치가유지되는지확인한다.
