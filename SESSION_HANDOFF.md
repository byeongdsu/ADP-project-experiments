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
