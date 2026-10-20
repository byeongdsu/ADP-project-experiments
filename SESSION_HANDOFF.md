# 작업 인계 — 2026-10-20

## 사용자 결정 (최우선)
EXP-001은 현재 결과와 비교 분석까지 정리한다. EXP-002~004의 기본 결과를 먼저 확보하고 문제별 파라미터 보완은 그 뒤 수행한다. 지금 PP 튜닝이나 m_l1 변경을 진행하지 않는다. EXP-002~004의 번호별 조건은 아직 확정하지 않았으므로 실행 전에 정한다.

## 목표와 역할
PP와 TLN의 한계를 GT/PF × 장애물0/1/2/4개 조건에서 관찰하고 이후 개선 가설을 검증한다. 사용자가 실행·관찰, 에이전트가 설정 변경·로그 분석·문서화를 담당한다.

## EXP-001 완료 범위
PP(C++) / GT / 장애물0 / hyhall12 / seed2026 속도 탐색 및 동일 경로 구간 비교 완료. 상세 대표 결과·주의사항은 runs/EXP-001/SUMMARY.md, 분석은 analyses/EXP-001/REPORT.md, 결과표는 results.csv.
고정출발0.75는 R06-retry4/5 두 실행에서 각각3랩 성공.0.85(R07-retry3)/0.95(R08)/1.0(R09)은 첫랩 충돌.공통s0.5~19.5m 평균|d|14.0/23.4/37.1/39.4cm. 네조건 초기GT pose/yaw/vx0 및목표경로배율 일치 확인.
기존R06은시작로그0.65로목표0.75미적용,유효0.75결과에서제외.초기조건이상이거나종료조작충돌인기록은각README에구분하고원본보존.3연속랩은독립반복3회가아니다.

## 현재 설정 (다음 실험에 자동 승계 금지)
/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/speed_scaling.yaml:global_limit=1.0.
PP m_l1=0.575 등 튜닝 파라미터는 미변경. 속도설정변경은base_system재시작필요.맵경로는소스맵을직접참조하므로배율YAML변경에빌드는불필요.
고정시작pose:x10.832046001482436,y-0.5021100154908403,yaw0.3652437316241457. hyhall12/start_pose.yaml을gym_bridge launch가읽어자동적용한다.기존실행을모두종료후시작하고RViz에서위치를다시찍지않는다.관련패키지빌드/테스트6개완료. configs/FIXED_START.md참조.

## 실행·기록
터미널마다race 사용(~/forza_ws/race_stack 이동+ROS underlay/overlay source).
1: ros2 launch stack_master base_system_launch.xml map_name:=hyhall12 pose_source:=gt
3: ros2 launch race_manager race_manager_launch.xml map_name:=hyhall12 seed:=2026 num_obstacles:=0 plot:=true log_dir:=/home/misys/ADP-project-experiments/runs/EXP-XXX/RXX
4: ros2 bag record -o /home/misys/ADP-project-experiments/runs/EXP-XXX/RXX/telemetry /drive /car_state/odom_GT /car_state/odom /car_state/frenet/odom /global_waypoints /local_waypoints /sim/collision
2: ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP
開始1→3→4→2、終了1→2→3→4。컨트롤러만먼저끄면마지막명령으로계속주행할수있다.다음실험조건에맞춰GT/PF/장애물수/배율을명시적으로선택한다. bag경로중복시덮어쓰지않는다.

## 후속 보완 가설 — 보류
속도비례lookahead 증가와제한적인횡오차/헤딩감속이원인후보.첫후보실험은0.85에서m_l1만0.575→0.50이나사용자결정에따라EXP-002~004이후로미룬다.실제L1/목표점미계측,설치설정과입력으로재구성한추정이며주원인확정금지.l1_distance는publisher선언만있고publish없음.

## 주의
racecar_version 기본NUC2,head_to_head는controller_cpp사용.컨트롤러설정과시뮬레이터SIM설정을혼동하지않는다.실제GT토픽은/car_state/odom_GT이며/ego_racecar/odom_GT가아니다.PF실험에서도GT를기록해위치추정오차와추종오차를분리한다.장애물실제배치/크기를보존한다.
race_manager report.py의--log-dir 및launch log_dir지원은구현/빌드완료.고정출발점/기록정비외의기존사용자코드변경을덮어쓰지말것.큰bag은로컬보관,Git에임의추가하지않는다.영상은아직미전달.GitHub인증비밀저장금지.
