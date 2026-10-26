# EXP-004/R09 — PP / PF / 0.70 / 장애물 4개

상태: 실행 준비. 실제 주행 결과 없음.

사용자 요청: EXP-004의 장애물 1개·4개 조건 추가. 기존 2개 R07과 비교하도록 배율0.70 유지.
조건: hyhall12, pose_source=pf, PP(C++), seed2026, 장애물4개, 한 변0.30m, 기존 랩별 배치 변경. PP m_l1=0.575, q_l1=0.04. 기존 고정 출발과 PF 시작 prior 유지.

관찰 기준: 3연속 무충돌 랩 / 첫 충돌 / 15초 이상 실제 정지. 정지명령 지속과 실제 정지를 구분한다. 3랩 이후 계속 주행하면 이후 결과는 별도로 기록한다. 연속3랩은 독립반복3회가 아니다.
출발 전 PF 검사 결과를 start_check.json에 저장하고 실제 개수·배치는 시작로그와 bag으로 확인한다.

실행 순서 1→3→4→2. 종료 순서 1→2→3→4. 각 터미널에서 먼저 race 실행. 조건 변경 시 모두 종료 후 새로 시작한다.

터미널 1:
```bash
ros2 launch stack_master base_system_launch.xml map_name:=hyhall12 sim:=True pose_source:=pf
```

터미널 3:
```bash
ros2 launch race_manager race_manager_launch.xml map_name:=hyhall12 num_obstacles:=4 seed:=2026 log_dir:=/home/misys/ADP-project-experiments/runs/EXP-004/R09
```

터미널 4:
```bash
ros2 bag record -o /home/misys/ADP-project-experiments/runs/EXP-004/R09/telemetry /drive /car_state/odom_GT /car_state/odom /car_state/frenet/odom /global_waypoints /global_waypoints_scaled /local_waypoints /sim/collision /pf/viz/inferred_pose /car_state/pose_GT /scan /state /perception/obstacles /sim/obstacles /sim/obstacles/markers /planner/avoidance/otwpnts
```

터미널 2:
```bash
python3 /home/misys/ADP-project-experiments/scripts/check_pf_start.py --output /home/misys/ADP-project-experiments/runs/EXP-004/R09/start_check.json && ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP
```
