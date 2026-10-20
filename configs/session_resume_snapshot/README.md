# 컨테이너 종료 전 재개용 소스 사본

2026-10-20 EXP-001·002 완료 시점에 실제 race_stack에서 사용한 고정 출발점·PF 시작 prior 및 관련 테스트·배율 파일의 사본이다. 원래 파일 경로를 디렉터리 구조로 유지했고 sha256.json에 해시를 기록했다. 현재 배율은0.80이다.

이 폴더는 실험 자료 백업이며 원래 ROS 패키지 설치 경로가 아니다. 다음 세션에서는 원래 작업 트리를 우선 확인하고, 사본을 무조건 덮어쓰거나 별도 사용자의 변경을 되돌리지 않는다. 수정된 패키지를 복구·변경한 경우 원래 작업공간에서 해당 패키지를 빌드하고 설치 경로를 확인한다. 초기화 관련 빌드 대상은 particle_filter,frenet_odom_republisher,stack_master이고 고정 출발점 launch는 f1tenth_gym_ros다.

기존 race_manager의 log_dir 지원 소스는 ../resume_snapshot에 보존돼 있다. race_stack 전체와 rosbag의 백업은 아니며 소스 저장소의 별도 커밋을 대체하지 않는다. 자세한 상태·다음 작업은 ../../SESSION_HANDOFF.md를 참조한다.
