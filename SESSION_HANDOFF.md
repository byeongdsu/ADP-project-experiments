# 再開 / 작업 인계

2026-10-20 중단. 사용자는 노트북 종료 예정.

## 목적
PP와 TLN의 한계를 GT/PF × 장애물 없음/1·2·4개 조건에서 관찰하고 파라미터 변경으로 원인과 개선안을 검증. 사용자가 실행·관찰, 에이전트가 설정 변경·로그 분석·문서화.

## 현재
EXP-001 = PP + GT + 장애물 없음 속도 탐색 묶음. R01 0.25(재실행 대표), R02 0.35, R03 0.45, R04 0.55(5랩), R05 0.65(재실행 대표), R06 0.75 모두 무충돌 완주. 나머지는 3랩.
R01 최초 실행 종료 조작 충돌은 평가에서 제외. R05 최초 실행은 벽에 박힌 초기 상태로 시작하여 제외. 원본 보존.

현재 race_stack/stack_master/maps/hyhall12/speed_scaling.yaml global_limit=0.75.
R05 평균 9.711초, R06 평균 9.724초로 거의 동일. 다음은 무조건 배율 증가보다 실제 속도 및 속도 제한·명령 적용을 확인할 것.

## 실행
1: ros2 launch stack_master base_system_launch.xml map_name:=hyhall12 pose_source:=gt
3: ros2 launch race_manager race_manager_launch.xml map_name:=hyhall12 seed:=2026 num_obstacles:=0 plot:=true log_dir:=/home/misys/ADP-project-experiments/runs/EXP-001/RXX
2: ros2 launch stack_master head_to_head_launch.xml ctrl_algo:=PP
시작 1→3→2. 종료 1→2→3 (컨트롤러만 먼저 끄면 마지막 명령으로 주행 지속).

## 코드 변경
race_manager report.py에 --log-dir 옵션 추가, launch에서 log_dir 전달, colcon build 완료. 관련 소스/launch 사본은 configs/resume_snapshot에 보관. race_stack의 기존 사용자 변경은 건드리지 말 것.

## 주의
기록기는 벽시계로 측정. 실제 속도·추종 오차 미기록. 3랩을 독립 반복 3회로 표현하지 않는다. PP C++ 구현이 실행된다. racecar_version 기본 NUC2, SIM 파라미터가 적용된다고 단정하지 말 것.

이 파일과 results.csv 및 runs의 README를 읽고 이어서 진행한다. GitHub 인증 비밀은 저장하지 않는다.
