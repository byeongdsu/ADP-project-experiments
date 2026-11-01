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
