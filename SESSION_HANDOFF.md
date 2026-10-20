# 작업 인계 — 2026-10-20

## 사용자 결정과 현재 상태
EXP-001(GT)과 EXP-002(PF)의 장애물 없는 기본 탐색 및 상호 비교를 마감했다. EXP-003~004를 먼저 관찰한 뒤 문제별 파라미터를 하나씩 검증·보완한다. 지금 미세 속도 최적화나 PP 튜닝을 하지 않는다. 이후 실험 번호별 조건은 아직 확정하지 않았다.

사용자는 결과 정리와 GitHub 업로드를 요청했다. 대상은 ADP-project-experiments 저장소다. race_stack 소스 저장소는 별도이며, 초기화 정비 패치·소스 기준 커밋·해시는 실험 저장소에도 보존한다. 원본 .db3는 기존 .gitignore로 제외하고 로컬에 남긴다.

## 먼저 읽을 자료
- runs/EXP-001/SUMMARY.md: GT 대표 결과와 예외 처리.
- runs/EXP-002/SUMMARY.md: PF 결과, 초기 측위 실패, 0.80 추가 탐색.
- analyses/EXP-001-vs-002/REPORT.md: 같은 배율·경로·GT 기준 비교.
- findings.md: 확인한 사실, 원인 가설, 보류한 검증.
- results.csv: 실행별 요약과 원본 경로.

## EXP-002 최종 관찰
알려진 초기 위치를 제공한 PF 조건에서 0.25/0.35/0.45/0.55/0.65/0.75는 각각 3랩 무충돌 완주했다. 평균 랩타임은 24.957/17.820/13.883/11.397/9.721/8.550초다.
- R07, 0.85: 0랩, 계측 +7.699초 벽 충돌 (11.437, 0.240).
- R08, 0.80: 계측상 1랩 8.120초 통과 직후 +8.129초 벽 충돌 (11.533, 0.258). 목표 3랩 실패다.
- EXP-002/R08은 0.80, EXP-001/R08은 0.95이므로 혼동하지 않는다.

초기화 정비 이후 각 bag에서 초기 GT pose·방향·속도0과 출발 전 정상 PF 오차를 확인했다. start_check.json은 각 주행 폴더에서 발견되지 않아 검사 명령 통과를 주장하지 않는다.

## GT/PF 비교
0.75와0.85의 네 실행에서 초기 GT pose·속도0과 글로벌 경로 형상이 같았다. 첫 주행의 공통 s0.5~41.5m를 0.5m 간격 82개 구간으로 정렬했다. 공간 평균 GT 실제 추종 거리는 GT0.75=15.81cm / PF0.75=11.94cm / GT0.85=27.40cm / PF0.85=22.69cm다.

해당 실행의 PF 값이 작았어도 PF가 GT보다 우수하다고 일반화하지 않는다. 추정 편향·난수·비동기 입력·실행 변동이 혼입될 수 있고 독립 반복이 충분하지 않다. GT/PF 0.85 충돌점은 약6.4cm 차이의 유사 코너다. PF만의 실패로 단정하지 않는다.

## 현재 설정 — 다음 실험에 자동 승계 금지
- /home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/speed_scaling.yaml: global_limit=0.80.
- PP 튜닝은 기존 값 유지: m_l1=0.575, q_l1=0.04 등.
- 고정 pose: x10.832046001482436, y-0.5021100154908403, yaw0.3652437316241457.
- base_system 재시작 시 맵의 start_pose.yaml이 적용된다. RViz에서 다시 위치를 찍지 않는다.
- 소스 맵 YAML을 직접 읽으므로 배율 변경에는 빌드가 필요 없지만 base_system 재시작은 필요하다.

## 초기화 정비와 과거 예외
EXP-002 최초 R01은 주행 전 PF 위치 오차3.909m·방향 오차179.702도인 상태에서 첫 명령 +0.805초에 충돌했다. 시뮬레이터만 고정 pose를 적용하고 PF에는 시작 prior가 전달되지 않았던 문제다. 잘못된 해로 수렴한 세부 원인은 미확정이며 실패 원본을 보존한다.

시뮬레이션에서만 start_pose.yaml을 PF 시작 prior로 한 번 전달하도록 정비했다. 기존 지역 입자 분산을 유지했고 주행 중 GT로 보정하지 않는다. 실차/localize 기본값은 전역 초기화다. 관련 테스트12개 통과, particle_filter·frenet_odom_republisher·stack_master 빌드 완료. 정지 검증 오차는 약1.6cm/0.07도다. configs/pf_initialization_validation에 패치·테스트·해시·검증 로그를 보존했다.

EXP-001 최초 R06은 목표0.75였지만 시작 로그는0.65였다. 유효0.75 결과에서 제외하고 원본은 보존했다. 수동 출발 차이, 초기 조건 이상, 종료 조작 중 충돌 등은 각 실행 기록에 구분했다. 고정 출발0.75는 두 별도 실행에서 각각3랩 완주했다.

## 실행과 기록 원칙
터미널마다 race로 폴더 이동과 ROS underlay/overlay를 적용한다. 시작 순서는1→3→4→2, 종료는1→2→3→4다. 컨트롤러만 먼저 끄면 마지막 명령으로 주행할 수 있다.

PF 조건은 pose_source:=pf, GT 조건은 :=gt를 사용한다. 매번 새 EXPID/RUNID 기록 경로를 선택한다. PF rosbag 토픽은 아래와 같다.

```text
/drive /car_state/odom_GT /car_state/odom /car_state/frenet/odom
/global_waypoints /local_waypoints /sim/collision
/pf/viz/inferred_pose /car_state/pose_GT /scan
```

PP 시작 전 `python3 /home/misys/ADP-project-experiments/scripts/check_pf_start.py --output <run>/start_check.json && ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP`를 사용한다. 실제 GT 토픽은 /car_state/odom_GT다. PF 기준 Frenet d와 GT 실제 추종 거리를 구분하고, 장애물 실험에서는 실제 배치·크기도 보존한다. 3연속 랩과 독립 반복3회는 구분한다.

## 분석 도구와 보류한 가설
scripts/analyze_pf_run.py는 PF 오차와 GT 실제 polyline 추종을 분리한다. 구간 시각은 각 README와 지표 JSON에 있다. analyses/EXP-001-vs-002/compare.py는 같은 배율의 첫 주행을 공통 경로 구간으로 비교한다. 재실행하려면 원본 bag을 해당 상대 경로에 준비한다.

원인 후보는 속도 비례 lookahead 증가, 제한적인 횡오차·헤딩 감속, 컨트롤러 속도 및 PF 위치 추정 지연이다. 실제 L1/목표점은 미계측이라 일부 해석은 소스와 입력의 재구성이다. l1_distance는 publisher 선언만 있고 publish 호출은 없어 이름만 bag에 추가해도 측정되지 않는다. m_l1=0.50 등 후속 검증은 EXP-003~004 이후로 보류한다.

## 보관과 주의
race_stack의 기존 사용자 변경을 덮어쓰지 않는다. 소스 변경은 별도 저장소에서 관리하고 초기화 정비 패치를 실험 저장소에도 보관한다. 큰 bag은 Git에 임의 추가하지 않으며 GitHub 인증 비밀을 저장하지 않는다. 촬영 영상은 아직 전달받지 않았다.
