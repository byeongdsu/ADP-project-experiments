# GitHub 게시 체크포인트 — 2026-11-18

현재상태통합문서 analyses/EXP-009/REPORT.md,추가17세션결과표 results_summary.csv/runs.json,재현 configs/EXP-009/REPRODUCE.ko.md. R27~R34는runs/EXP-008, Berlin B01~B07와미실행2개는runs/EXP-009. 원본23.08GB신규bag은로컬만/기존bag도보존. 사용자berlin_map커밋a34fb4b를fast-forward로보존하고그위에보고서·기록게시. race_stack소스저장소commit/push아님. 아래작업중GitHub미게시문구는해당시점의과거상태. 신규주행·B07수정·PPO학습시작없이현재실패/회귀를그대로정리했다. 다음은B07빌드/검사경로회귀수정과동일입력검증,후보실패사유분리및저속재계획. SVG개선/안정완주/PPO성과미확인.

# 최신 결과 — 2026-11-18 B07 guard 정지/계산 회귀

B07주행완료:0랩/충돌0/10.264m/9.071초관측,reason guard_stop:no_collision_free_forward_candidate. 비영속도시작6.967초후첫장애물부근정지,역주행관측없지만B06 U턴장면전약10m에서끝났으므로역주행회피완주성공아님. guard후.231초/약.127m로|v|<.05,종료GT거의0. 새cyclemean135.031/p95231.388ms/25ms초과100%,기존B06mean24.119보다큰시간회귀. coreO3/node최적화없고공유inline검사weak함수비최적화경로확인. 동일core/mainO0대O3진단고정입력실행1.568대.685초(유한빈맵1회,실주행전체회귀원인단정금지). 새검증구현회귀이며원본SVG본래한계로서사화금지. 이번분석은실험binary/설정변경없음. 세션REVIEW.ko.md,direction_detail.png,guard_stop_detail.json. 다음빌드/검사호출회귀수정과출력동등성검증먼저,이후정지입력에서방향/충돌불합격계수분리및모델-명령일치저속재계획검증. PPO/가변속도미구현. 기존원본보존.

# 최신 준비 — 2026-11-18 B07 전방 진행/최종검증

사용자승인으로별도ws_forward_guard/forward_guard_v1구현. 기존footprint보존. 양쪽공통:역방향/후방투영/충돌후보최종가중치0,진행거리보상-2/m,평균제어재예측실패시최대가중치유효후보대체,발행전2cm/2도보간검증. 실패시guard_stop 목표0/이전조향반복발행/latch(자동재출발없음). 규칙가변속도재계획/PPO아직없음. runner guard_stop후|v|<.05가1초유지면종료/사유summary보존. 소스/도구before,diff보존.
빌드/단위검사/각solver15쌍재현/전점유거부/합성ROS두모드zero명령+latch/Berlin정지입력검사통과. 오프라인Gym1.5->0검사.22초/.126m로|v|<.05,지연없는유한테스트로안전보장아님.
다음수동SVG세션 manual_runs/20261118T142140_80314_svg_mppi_B07_berlin_svg_forward_guard_speed1p5. B06와YAML물리설정해시동일,명목1.5/GT/4obs/seed2026/7lap/600s. 실제주행미실행. 이번목표역주행/부적합궤적계속실행억제,보수적정지증가가능/정지는회피성공아님. 정확한근거ROOT/diagnostics/forward_guard_20261118/REVIEW.ko.md. 기존race_stack변경안건드림/GitHub미게시.

# 최신 결과 — 2026-11-18 B06 SVG 역주행

B06사용자중단,0랩/충돌0/약31.38초. 첫비영속도명령+17.666초headingerror-90.706도,우회전포화-.4189,候補충돌비율.997,양의속도1.5. 이후거의180도역방향약13초,nearest참조진행량최대에서17.49m감소. 음수속도없음,초기정렬정상. 약16.482초선택예측45상태중24개가이미역방향. 공유비용에전방진행/역방향hard조건없고q_angle.01. SVGguide/평균화개별기여미분리. cyclemean24.119/p9537.616ms,25ms초과34.43%. B05보다개선확인못함. 기록/그림/원인범위세션REVIEW.ko.md,direction_detail.json/png,predicted_direction.json. 코드/설정변경/신규주행없음. 다음공통진행방향조건과불가시감속처리검증을우선권장,RL/속도상향전에구조해결.

# 다음 실행 준비 — 2026-11-18 B06 SVG

사용자 동일조건SVG진행승인. 세션 /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261118T140922_71360_svg_mppi_B06_berlin_footprint_svg_speed1p5 준비, 주행미실행. B05와 동일footprint_v1 바이너리/물리설정/지도/GT/속도1.5/4장애물/seed2026/7랩/600s. YAML값변경은mpc_mode forward_mppi->svg_mppi만, 기존SVG세부설정유지. 비교확인 comparison_conditions.json. 실행1->3->4->2, 종료2자동후1->3->4. 자동감속/PPO아직없음. 이전B05결과/원본보존.

# 최신 결과 갱신 — 2026-11-18 B05 주행 분석

B05(20261118T140204_66033_forward_mppi_B05_berlin_footprint_speed1p5) 사용자주행완료:1랩33.049초무충돌후2번째랩벽충돌,목표7랩실패. runner+51.135초,76.225m. 첫장애물배치B03와동일,랩변경후새배치코너에서실패. 마지막2초후보충돌비율평균.99186인데속도명령1.5유지. cyclemean14.922/p9522.302/max37.056ms,25ms초과3.082%. 종료speed0/bagmetadata정상/활성주행없음. 새geometry에진전있으나1회로안정화단정금지. 세션REVIEW.ko.md와comparison_after_run.json/collision_detail.png보존. 다음공정SVG대조는footprint_v1로해야함(기존B04는공통비용이다름). 이후위험감속/PPO속도선택. 이번분석중새주행/설정변경/학습없음. 아래B05미실행기록은이갱신이전상태.

# 최신 체크포인트 — 2026-11-18 Berlin 공통 충돌 검사 / RL 허용

사용자 발표12월16일. 시뮬레이션 한정, 강화학습/속도 변경 허용. 현재 주행·학습 자동 시작 안 함. 과거 아래 11월1일 기록보다 실제 shared_dir 진단/세션이 최신임.

- ROOT=/home/misys/shared_dir/svg_mppi_ros2_20261031. Berlin 교수 파일 사용자 복사 위치 race_stack/stack_master/maps/berlin. 지도/경로/시작 위치 통합 완료. speed_scaling은 해시 기록만, 현재 MPPI fixed speed에 적용 안 함.
- consistent_v1 전역/로컬 초기조향 일치 수정 후 HYHALL3m/s 5회35랩 성공 기록 있음. Berlin B01/B02 1m/s, B03 forward1.5m/s, B04 SVG1.5m/s 모두 첫랩벽충돌. B04 실제 세션20261118T134531_53720, 이전131317_46412는 미실행. SVG gradient 병렬 누적 race를 ws_svg_repro에서 수정해 고정입력 재현 검증했으나 Berlin 주행 개선은 확인 못 함. 둘 다 공통 비용/입력 분석 필요.
- 사용자 승인한 다음 수정: 새 ws_footprint, solver-version footprint_v1. 현재 ego 축 고정 팽창맵/중심점 조회를 미래yaw 회전 안전 직사각형(.70x.30m) 대 raw scan 점유 셀 교차로 대체. softmargin .30/peak5 유지하되 사각형 간 거리 정의. 전체 차체 grid 밖100, 종단도 동일. 기존 variant/bag/설정 보존. 기존 race_stack dirty 변경 안 건드림. 이것은 기본 알고리즘 개선 증명이나 최종 궤적 hard reject/자동위험감속 추가가 아님.
- 빌드 통과, 독립geometry12006조건불일치0, forward/SVG각15고정입력쌍불일치0. domain78 정지검사passed, scan/GT/reference489/맵600x600/raw_occupancy/4obstacles/ready_disabled, 비영속도명령0.
- 사용자 다음 실행용: manual_runs/20261118T140204_66033_forward_mppi_B05_berlin_footprint_speed1p5. B03 대비 controller.yaml/simulator.yaml 및 map hashes/seed2026/4obs/7laps/600s 동일, fixed1.5m/s GT. 주행 아직 미실행. 준비중 생성140028_64436은 NOT_EXECUTED로 표시, 사용금지.
- 정확한 수동 명령은 새세션 TERMINALS.txt. 순서1(sim)->3(scenario)->4(record)->2(controller). 2 종료 후1->3->4 Ctrl+C, recorder prompt 대기. 예전세션재사용금지. ROS2 Humble 기존컨테이너 터미널 사용.
- 새 변경/검증/일정은 ROOT/diagnostics/footprint_20261118/REVIEW.ko.md, RL_PLAN.ko.md, source_manifest.json. RL 추천은 MPPI조향+PPO목표속도. 실제cmd와예측속도일치하는가변속도인터페이스부터구현필요. 우선고정속도충돌검증->규칙감속->PPO 비교. RL 미구현/미학습. 발표직전검증기간확보.
- 이번추가작업은로컬만;11월1일이후결과/코드를GitHub게시했다고말하지않음. 원본bag보존, EXP005복원금지.

# 최신 체크포인트 — 2026-11-01 MPPI 분석·GitHub 게시

사용자 요청: 3.0m/s 성공/실패 분석과 PP→MPPI→SVG 연구 흐름을 GitHub에 정리. analyses/EXP-008/REPORT.md가 최신 통합 보고서이며35개 세션 자료/결과표/소스 변형을 보존했다. 원본 bag 약25GB는 /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs에만 보존. 이것은 실험 기록 저장소 게시이며 race_stack 소스 commit/push가 아니다.

- PP의 확인된 원인: GT 2장애물에서 회피경로가 두 번째 장애물 점유검사에 걸려 O_FREE False→TRAILING→속도0. PF0.70은3랩통과후4번째배치충돌도 있으므로 PP는 항상0랩이라고 쓰지 않는다. PP최고7.014m/s는고정속도와다르며GT/PF혼합비교금지.
- MPPI ROS2별도어댑터: 원본ROS1커밋 f4fb7b74f12d4568790213aab26dd6f0538d1925. repro_v1 sampler공유분포수정, speed_v1상한3, clearance_v1추가여유비용(.30m/5). MPPI가직접조향,PP결합아님,고정속도이며자동위험감속없음.
- 2.5m/s 장애물2 R15~R17각7랩성공,과거R13충돌도보존. 장애물4 baseline R18실패/R19R20성공. clearance R21retry/R22/R23목표7랩성공이나R22종료후벽충돌별도기록. 종료정지반복송신수정후R23정지확인.
- 최신3m/s clearance R24는2랩벽충돌/R25는7랩성공/R26은4랩벽충돌. 동일조건3회중1회목표성공. 추가반복보다예측/실제회전/조향반전과최종궤적안전검증부재분석필요. 새진단그림/JSON은EXP-008/diagnostics/speed3_failure_review_20261101.
- 추가합성검사:전역초기steer추정값/로컬초기steer0불일치. steer.3/속도3에서전역첫점의5mm단일grid비용100인데solver충돌비용0인좌표불일치재현.실제충돌인과는미확정.공통MPCBase이므로SVG전환전양쪽동일초기상태검증필요.실주행solver수정은아직없음.
- 후보평균화/동역학불일치의개별기여미확정. 미래실제는재계획결과이므로고정예측과의차이를모델오차단독으로단정금지.
- SVG는후속가설,실행시간단축보장없음. S01예비SVG실패기록도보존하되현재공정비교자료아님. 사용자는오늘기본MPPI만,연산최적화보류,RL/PPO이후단계. 새시뮬레이션자동시작없음.
- 다음: 보고서/원본조건을읽고후속진단·단일변수개선또는사용자선택시공정SVG비교. EXP-005고정배치복원금지. 기존race_stack변경덮어쓰기금지. 수동터미널절대경로선호.

아래는 과거 인계다.

# 최신 체크포인트 — 2026-10-30 TLN 실험 마무리

사용자 요청: TLN 튜닝으로 확인한 한계를 PDF용 원고로 정리하고 실험 기록 저장소에 게시한다. 최종 문서는 analyses/EXP-007/REPORT.md. R14까지 결과18개, R02미실행. R13·R14동일장애물부근충돌확인. 추가튜닝·실험자동실행없음. 원본bag로컬보존. 공유race_stack소스의commit/push와구분한다. 아래10월21일인계는과거기록이다.

# 작업 인계 — 2026-10-21 종료

이 문서 상단이 최신 상태다. 오늘은 사용자 요청으로 종료하며 자동 실행하지 않는다.

## 다음 세션에서 바로 할 일
1. 이 파일과 아래 분석 README를 읽고 기존 bag을 오프라인으로 분석한다. 실험을 처음부터 반복하지 않는다.
2. GT 첫 배치에서 회피경로가 두 번째 장애물의 점유 검사에 걸리는 과정과, PF 네 번째 배치에서 회피방향이 급변하는 과정을 기존 토픽만으로 비교한다.
3. 당장 추가 속도 탐색·seed 변경·새 진단 노드·고정 배치 기능을 추가하지 않는다. 사용자와 다음 검증 조건을 정한 뒤 실행한다. 특히 EXP-005 기능은 사용자 요청으로 원복했으므로 자동 복구/재적용 금지.
4. results.csv에는 오늘 실행 일부만 반영됐다. EXP-003 R03/R05/R06/R07계열 등의 누락과 run README를 결과JSON·시작로그·bag으로 점검해 보완할 수 있다. 완료하지 않은 분석을 완료라고 표현하지 않는다.

재개 메시지 예시: **“ADP-project-experiments의 SESSION_HANDOFF.md 읽고, EXP-006까지의 기존 bag으로 회피 경로 문제 분석부터 이어서 해줘. 코드는 먼저 바꾸지 마.”**

## 현재 실제 설정과 소스
- global_limit=0.70, hyhall12, PP(C++), seed2026, 장애물2개. 마지막실행은 GT. pose_source는 launch 인자이며 다음실행에 명시한다.
- PP 튜닝값은 기존그대로(m_l1=0.575,q_l1=0.04). 고정출발/PF시작prior는 이전세션에서 정상화한것을 유지.
- EXP-005 고정배치·state_machine진단추가분은 모두 제거. state_machine 소스6개와 설치파일을 변경전 build/lib사본과 바이트대조했고 기존 PP설정도 일치. race_manager/state_machine은 일반설치로 재빌드. 설치에남은 fixed_layout.py도 제거.
- 확인근거 configs/EXP-005/rollback_verified.json. configs/EXP-005/source_snapshot은 폐기된실험소스의기록이므로 실행용아님. 그 폴더 COMMANDS.md를 사용하지 않는다.
- race_stack의 git diff에는 기존 사용자·이전세션 변경이남아있다. git reset/일괄복구 금지. 오늘종료시 source상태는 configs/session_close_20261021에별도보존.
- 원본bag로컬유지. 오늘종료정리는 로컬파일저장이며 GitHub push는 수행하지않았다.

## 핵심 관찰
- EXP-001/002는 이전세션에서마감: 장애물없이 GT/PF0.75대표완주,0.85유사코너실패. 다시실행하지않음.
- EXP-003(GT): 장애물1개0.25는3랩완주(평균25.306초). 장애물2개0.25~0.75는정지관찰. 조건별원본재검토필요. analyses/EXP-003/REPORT.md.
- EXP-004(PF): 2개,0.25/0.35/0.45/0.55/0.65는첫구간정지. 0.75최초는1랩완료후새배치충돌; 반복은정지(약9초/19.85초관찰). 정확한관찰길이차이를보존.
- PF0.70 R07/R07-retry1/R07-retry3는각각3랩무충돌후4번째새배치에서장애물충돌. 랩평균12.374/15.116/14.098초(반올림). 목표3랩성공과전체충돌1회를분리. R07-retry2만별도로시작직후벽충돌(계측+0.589초),원인미확정.
- EXP-006(GT0.70): R01과R01-retry1모두첫배치 O_FREE False→TRAILING,0랩무충돌. R01은18.45초정지, retry1은8.325초후기록종료해15초기준미충족. 네번째배치에는도달하지못했음.

## 원인별 증거 / 미확정 구분
1. 첫배치: 첫장애물회피경로가두번째장애물에가까워져 `_check_ofree`의전방3m/횡차이0.1m기준에걸림. 같은시각O_FREE False와OVERTAKE→TRAILING확인. TRAILING간격제어속도0,FTG는기존코드에서비활성화.
2. 네번째배치: R07-retry3 충돌0.3→0.2초전 planner right→left. 동일s21.9부근local d -0.541→+0.472m(약1.01m급변),ego d약-0.37m유지. 이후정지명령에도실제속도가남아충돌. 정확한방향변경트리거는아직미확정.
3. 단기경로보존은존재: 빈planner경로는즉시기존경로를지우지않음, 최근유효경로를TRAILING에서도사용,마지막유효경로부터TTL설정3초(루프카운터)지나면삭제. 이는지난랩학습의근거가아님. 새planner경로자체도좌우변경되므로캐시하나만을충돌원인으로단정하지않음.
4. PF가GT보다우수하다고일반화금지. 추정편향·난수·입력지연·경로/상태전환변동이있고독립반복수부족.

## 우선 읽을 분석
- runs/EXP-004/R07-retry3/README.md 및 path_switch_timeline.json
- runs/EXP-004/R07/README.md, R07-retry1/README.md
- runs/EXP-004/R06/README.md 및 collision_trajectory.png
- runs/EXP-006/R01/README.md 및 R01-retry1/README.md
- analyses/EXP-003/REPORT.md
- scripts/analyze_obstacle_collision.py, scripts/analyze_pf_run.py

분석은bag수신시각최근이전표본결합이며내부콜백시각과차이가있다. waypoint점거리와polyline거리구분,글로벌레이스라인거리와회피경로거리구분. 장애물개수뿐아니라실제XY/yaw·크기·랩별변경확인. PF start_check.json 없는실행은통과주장금지. 영상촬영통보만있고영상파일은미전달.

## 실행·사용자 선호
- 사용자가속도변경을요청하면파일적용후새run경로와터미널4bag명령도함께알려준다. 터미널3 log_dir는직접절대경로표기를선호.
- 실행1→3→4→2,종료1→2→3→4. 각터미널race로환경적용. 배율변경후base_system재시작필요. PF는출발전check_pf_start.py && PP로검사결과저장. GT는GT입력명시.
- RViz는호스트X접근권한문제발생이력있음. 컨테이너DISPLAY=:0,XAUTHORITY=/dev/null이었음. 설치오류와X권한오류를혼동하지않는다.
- 소스변경후실행오류로사용자불편이컸음. 다음은읽기전용분석부터,불필요한수정/재설치/동작정상확신을피한다.

이전상세인계는 analyses/session_history/handoff_before_20261021_close.md에보존했다.
