# PF 시작 초기화 정비

EXP-002/R01에서 PP 첫 명령 전에 PF-GT 오차3.909m/179.702도가 발생했다. 시뮬레이터만 고정 pose를 적용하고 PF는 전역 초기화했던 절차 문제를 수정했다. PP 튜닝이나 센서·운동모델 계수는 바꾸지 않았다.

## 구현
- simulation base_system→Frenet launch→localize launch에 `initialize_from_start_pose=true`를 전달한다. 실제 차량/직접 localize launch 기본값은 false다.
- 맵 디렉터리 start_pose.yaml에서 시뮬레이터와 같은 x/y/yaw를 PF의 시작 prior로 한 번 전달한다. PF는 기존 지역 초기화 함수(위치표준편차0.5m,방향0.4rad)를 사용한다. 분산을 줄이거나 PF를 GT로 계속 보정하지 않는다.
- 파일이 없는 맵은 전역 초기화로 유지하고,NaN/무한 pose는 거부한다.
- simulation readiness 스크립트 `scripts/check_pf_start.py`는 읽기만 한다. GT정지속도<0.05m/s,PF-GT위치오차<0.30m,방향오차<15도가2초 연속 유지될 때만 exit0을 반환한다. 최대30초 후 실패하면 PP를 시작하지 않는다. 기준은 초기화 이상을 걸러내는 실험 절차 기준이지 정상주행오차 허용치나 수렴 확률 보장이 아니다.

## 검증
초기 prior/기본 fallback/실제 차량 모드/잘못된값 테스트와 기존 맵선택 테스트 총12개 통과. particle_filter,frenet_odom_republisher,stack_master 빌드완료.
별도 ROS domain47에서 PP 없이 정지 센서추정 약5초 확인: 마지막30표본 평균위치오차0.01593m,방향오차0.0752도. domain48에서 실제 readiness 스크립트 통과:위치0.01587m,방향0.0659도,GT속도0. 주행은 수행하지 않았다. headless 검증에서 RViz 렌더러 오류가 기록됐으나 시뮬레이터/PF와 센서추정은 실행되어 값을 수신했다. 사용자 GUI 동작을 검증한 것은 아니다.

## 재실행
기존 실행 모두 종료,base_system을pose_source:=pf로재시작한다. EXP-002/R01-retry1에결과와bag기록을먼저시작한뒤터미널2에서다음명령을실행한다.

```bash
race
python3 /home/misys/ADP-project-experiments/scripts/check_pf_start.py --output /home/misys/ADP-project-experiments/runs/EXP-002/R01-retry1/start_check.json && ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP
```

PF READY일때만PP가시작된다.속도0.25유지,고정출발점RViz수동변경없음.수동/주행중/initialpose재설정은하지않는다.실패R01은삭제하지않고초기측위실패로보존한다.이번정비후의실험은'초기위치를알려준PF추종'조건이며전역위치인식성공을평가하는조건과구분한다.

소스변경패치 initial_prior.patch,검증정량JSON과실행로그를보존했다.추후GitHub에실험문서를올려도race_stack소스변경은별도저장소임을기억한다.
