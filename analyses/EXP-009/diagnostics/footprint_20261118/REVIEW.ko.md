# 2026-11-18 공통 충돌 검사 분리 버전

사용자 승인: 시뮬레이션만 진행. 예측 차량 방향을 반영하는 충돌 검사 수정 후 Berlin 동일 조건 비교. 발표 2026-12-16, 강화학습/속도 변경 허용. 자동 주행/학습은 시작하지 않음.

## 변경

기준 ws_svg_repro의 src를 새 ws_footprint에 복사. 기존 소스/바이너리/설정/bag/지도는 보존. 사용자 race_stack 변경은 건드리지 않음. manual_session.py에 footprint_v1 선택지만 추가했고 변경 전 파일과 diff 보존.

기존: 현재 ego 축으로 raw scan 점유 격자를 15x7 커널로 부풀림. 모든 미래 상태의 중심점을 같은 collision_layer에서 조회. 미래 yaw는 이 조회에 반영되지 않음.

신규: scan 원점/해상도/처리 주기는 유지하고 raw_occupancy layer를 추가. 각 미래 상태의 x/y/yaw로 회전하는 안전 직사각형과 점유 격자 사각형(.05m)의 교차를 SAT로 판정. 직사각형 .70x.30m는 차체 .58x.2032m에 기존 의도한 각측 여유 .06/.0484m를 더한 크기. 종단과 중간 상태 모두 적용. 기존 raster dilation과 셀 경계 수치까지 동일하다는 뜻은 아님.

여유 비용은 두 사각형 사이 유클리드 거리 d에 5*max(0,1-d/.30)^2. 기존 .30m/5는 유지하되 기존 팽창 raster의 EDT와 거리 정의가 다름. 충돌 비용100과 여유 비용은 별도 합산하여 collision_rate 의미 유지. 전체 차체가 local grid 밖이면100 (기존 중심점/종단 일부10과 다름). legacy collision_layer/clearance_cost는 입력/시각화 호환용으로 그대로 발행하나 새 solver가 사용하는 비용과 일치하지 않으므로 RViz 음영을 새 비용 자체로 해석하지 않음.

기존 샘플러 수정/전역·로컬 초기 조향 일치/SVG gradient 고정 순서 reduction은 계승. 조향 모델/속도/가중치/샘플 수/SVG 옵션은 이번에 튜닝하지 않음. collision_rate_threshold에 따른 자동 정지, 최종 평균 궤적 hard reject, 속도 최적화는 추가하지 않음. 자세 의존 검사도 scan 비가시 영역과 이산 시간 사이의 충돌을 보장하지 않음.

## 검증

- colcon 빌드 성공. build.log/build.final.log.
- 독립 다각형 edge/vertex oracle와 12,006 조건: collision 불일치0, 여유 비용 최대 오차 7.11e-15. 접촉 경계 부동소수점 오차는1e-12 허용으로 수정 후 재검사.
- 실제 ForwardMPPI/SVGuidedMPPI 연결, 각15 고정 입력 쌍, thread4: control 불일치0, nonfinite0. 이것은 궤적의 주행 안전/성공 판정이 아님.
- 정지 상태 ROS domain78: 600x600 Berlin, reference489, reference speed1.5, GT 출발 좌표일치, obstacles4, raw/legacy cost layers, ready_disabled, drive publisher1, /drive publisher0, nonzero command0. stationary_check.json.
- 최초 준비 세션 20261118T140028_64436은 검증 중 바이너리 갱신으로 주행용 아님. NOT_EXECUTED.md 표시. 새 수동 세션을 생성함.

## 다음 주행

manual_runs/20261118T140204_66033_forward_mppi_B05_berlin_footprint_speed1p5

B03와 controller.yaml 및 simulator.yaml 바이트 동일, map_asset_sha256/seed/obstacles/laps/duration 동일. solver workspace와 공통 비용 정의만 변경. Berlin/GT/fixed1.5m/s/45states*.05s/4 obstacles/seed2026/7laps/600s. speed_scaling global_limit.50은 파일 기록만 하며 fixed speed 구동에 적용하지 않음.

실행 순서 1 sim ->3 scenario ->4 record ->2 controller. 각 절대 경로 TERMINALS.txt. 터미널2 종료 후 나머지는1->3->4 Ctrl+C, bag prompt까지 기다림. 주행 개선·완주·랩타임은 아직 미측정.
