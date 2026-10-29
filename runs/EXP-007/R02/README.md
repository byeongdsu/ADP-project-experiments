# EXP-007/R02 — TLN / PF / 장애물 1개 (실행 준비)

아직 실행 결과 없음. R01 대비 장애물 2개→1개, 속도 매핑 범위 3.0~6.5→1.5~3.25m/s로 변경한다. 장애물 개수도 달라져 R01과의 차이를 속도 효과만으로 해석하지 않는다. 이후 속도 비교는 장애물 1개로 유지한다.

- map: hyhall12
- pose_source: pf (TLN 직접 입력은 /scan)
- seed: 2026
- num_obstacles: 1
- obstacle_size: 0.30m (현재 설정; 실행 중 marker로 재확인)
- model: TLN_aug.keras
- TLN 속도 범위 0.5배: v_min=1.5, v_max=3.25. 나머지 유지: curve_speed_weight=0.65,
  ema_alpha=0.7, steer_abs_max=0.36, clamp_range_max_m=10.0, down_sample=2
- PP global_limit=0.70은 TLN 속도에 적용되지 않음.
- 장애물 개수 변경으로 배치 추첨도 달라질 수 있음. R01과 동일 좌표라고 가정하지 않는다.
- 시작 1→3→4→2, 종료 1→2→3→4. 각 터미널에서 race 먼저 실행.

터미널1:
```bash
ros2 launch stack_master base_system_launch.xml map_name:=hyhall12 racecar_version:=SIM sim:=True pose_source:=pf
```
터미널3:
```bash
ros2 launch race_manager race_manager_launch.xml map_name:=hyhall12 seed:=2026 num_obstacles:=1 log_dir:=/home/misys/ADP-project-experiments/runs/EXP-007/R02
```
터미널4:
```bash
ros2 bag record -o /home/misys/ADP-project-experiments/runs/EXP-007/R02/telemetry /drive /scan /car_state/odom_GT /car_state/pose_GT /car_state/odom /car_state/frenet/odom /pf/viz/inferred_pose /global_waypoints /sim/collision /sim/obstacles /sim/obstacles/markers /sim/lap_count
```
터미널2:
```bash
python3 /home/misys/ADP-project-experiments/scripts/check_pf_start.py --output /home/misys/ADP-project-experiments/runs/EXP-007/R02/start_check.json && ros2 run ofc tln_inference --ros-args --params-file /home/misys/ADP-project-experiments/runs/EXP-007/R02/tln_params.yaml
```

## 속도 실험 순서

R02는 장애물 1개 / 속도 범위 0.5배의 첫 기준 기록이다.
같은 조건 독립 반복 후 필요하면 별도 run에서 0.7배(v_min=2.1,v_max=4.55)를 비교한다.
실제 속도와 충돌/완주를 확인한 후 다음 조건을 정한다.
모델, seed, 장애물 개수, 조향 관련 값은 고정한다. 실제 배치는 bag으로 대조한다.
