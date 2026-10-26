> 중단·원복: 사용자 요청으로 EXP-005 고정 배치·진단 기능을 제거했습니다. 아래 내용은 과거 준비 기록이며 실행하지 마세요.

# EXP-005 고정 배치 재현 및 진단

사용자 승인: EXP-004 네 번째 주행의 충돌 배치를 고정하고 저장회피경로와 상태전환 원인을 계측한다. 기존 PP·플래너 튜닝과 경로보존 로직은 변경하지 않았다.

## 조건
PP / PF / hyhall12 / 배율0.70 / 장애물2개(한 변0.30m). seed2026을 기록하되 고정모드에서는 추첨하지 않는다. fixed_layout.json에 EXP-004/R07의 네 번째 배치 실제XY·yaw를 보존했다. R07-retry1의 동일배치와 대조했다. 매랩교체하지않는다. 원래고정출발및시작prior 유지.

이 시험은 이전3랩의 측위·경로상태 이력을 생략한다. 같은배치가첫주행에서도문제를일으키는지 먼저 확인하며, 과거4랩충돌과동일초기조건이라고주장하지않는다. 여기서충돌이재현되지않으면 원래랩별배치조건에서도추가진단토픽을기록해 비교한다.

## 구현
race_manager에 선택적fixed_layout_file 인자추가. 기본빈값에서는기존배치방식유지. JSON의맵·개수·크기·유한좌표검사, 랩변경시고정좌표재발행. 파일을실행중다시읽지는않는다.

/state_machine/active_avoidance_wpnts: 매제어주기저장유효회피경로전체. TTL만료직전기준이며 /local_waypoints와함께사용해실제사용구간을판단한다.
/state_machine/diagnostics: 현재/이전상태, 전환이유, 실제평가된guard결과(미평가항목은생략), 점유를일으킨장애물ID/s/d/경로d/임계값, TTL카운터, 마지막planner수신정보(빈경로포함), 저장경로ID와점수·사용여부. guard를계측목적으로다시호출하지않는다.

신규토픽으로CPU/기록부하가추가되므로기존실험과완전히동일타이밍이라고주장하지않는다. 원래동작수정은하지않았으며이실행부터계측버전을구분한다.

## 검증
고정배치/맵·크기·개수·유한값거부 및기존배치·스코어검사25개, 경로보존과상태전환진단검사4개, 총29개통과. race_manager/state_machine 빌드완료. ROS domain87에서bridge/controller없이고정배치가lap_count=3후에도불변임을확인. 저장회피경로와diagnostics각20표본수신. validation.json과로그보존. 실제주행·충돌재현검증은미수행.

source_snapshot/은실제수정파일사본이며source_sha256.json으로식별한다. 기존race_stack사용자변경이있는작업트리에서작업했으므로전체작업트리를이패치의변경이라고해석하지않는다.

## 실행
runs/EXP-005/R01/COMMANDS.md 참조. 결과기준3랩·첫충돌·15초연속정지. 출발전PF검사통과를필수로기록한다. 원본bag은로컬보존.

## 설치 복구
사용자 터미널에서 symlink-install 이후 PackageNotFoundError가 반복되어 race_manager/state_machine의 build·install 산출물을 /tmp/exp005_install_backup_1792564982856358827로 보존하고 일반설치로 재빌드했다. 빌드폴더를 sys.path에서 제외한 상태에서도 설치디렉터리의 egg-info 및 모든 console entry point 로드 확인. domain87 고정배치·진단 검증도 다시 통과. 소스 변경 없음.
