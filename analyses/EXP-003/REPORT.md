# EXP-003 장애물 2개 정지 원인

## 확인된 원인
0.25, PP, GT, hyhall12, seed2026. R02와 R02-retry2에서 무충돌 정지가 반복됐다. R02-retry2의 OVERTAKE → TRAILING 전환은 경로 점유 검사 `_check_ofree` 실패다.

- state_machine.log: 1792560412.650639028에 `O_FREE False, obs dist to ot lane: -0.049370979178987334 m`.
- /state: 1792560412.6510417에 TRAILING. 두 수신 시각은 약0.4ms 차이다.
- bag 최근 이전 /local_waypoints·/perception/obstacles·Frenet을 결합한 근사 재구성: ego s=21.98167m. 두 번째 장애물 s=24.95513m, d=-0.14088m. 해당 s 부근 경로 d=-0.19088m, 횡방향 차이 약-0.05000m. 첫 장애물에서는 약-0.67938m다.
- 두 번째 장애물이 전방 3m 검사 범위에 진입하면서 횡차이 절댓값이0.1m 미만이 되어 점유 판정된다. 이는 내부 판정과 bag 재구성이 일치하는 근거이며 센서의 정밀 동기화나 물리적 충돌 여유를 뜻하지 않는다.

## 정지로 이어지는 구조
`SpliniOvertakingTransition`은 o_free=False이면 TRAILING을 반환한다. PP의 TRAILING은 정적 장애물 속도0과 간격 오차를 사용해 속도를 계산하고 음수를0으로 제한한다. /drive에서 끝6.5초간0을 확인했다. FTG는 ftg_disabled=True라 정지 후 자동 탈출하지 않는다.

플래너 `do_spline`은 가장 가까운 장애물 하나를 중심으로 회피 spline을 만든다. 상태기계는 그 회피 경로에 대해 전방의 다른 장애물도 검사한다. 따라서 첫 장애물의 회피 경로가 두 번째 장애물을 가로지르면 회피가 거부되고 정지할 수 있다. 이번 관측이 그 구조와 일치한다. 단순히 속도가 낮아서 생겼다는 근거는 없다.

## 범위와 다음 단계
장애물4개와 속도증가는 사용자 결정으로 보류. PP 튜닝, 점유 임계값 완화, FTG 활성화 등 동작 변경은 하지 않았다. 검사 결과는 현재 경로를 사용할 수 없다는 뜻이며 맵 전체에 통과 가능한 경로가 없다는 증명은 아니다.

추가 진단 실행이 필요하다면 기존 토픽에 /planner/avoidance/otwpnts 및 /global_waypoints_scaled를 추가해 생성 spline과 점유 검사를 직접 비교한다. 우선 보완 후보는 여러 장애물을 함께 고려하는 경로 생성이다. 점유 검사만 완화해 통과시키는 방법은 검증된 해결책이 아니다.

영상은 사용자 촬영완료 통보만 받았고 파일은 미전달. 분석은 로컬 bag·로그·소스에 근거한다.

## 근거
- runs/EXP-003/R02-retry2/state_machine.log
- runs/EXP-003/R02-retry2/stop_diagnostics.json 및 telemetry/
- state_machine/state_machine/transitions.py: SpliniOvertakingTransition
- state_machine/state_machine/state_machine.py: _check_ofree
- planner/local_planners/spline_planner/spline_planner/spline_planner.py: do_spline
- controller_cpp/src/pp.cpp: trailing_controller
