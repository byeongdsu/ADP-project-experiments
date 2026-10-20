# hyhall12 고정 출발점

이후 실행은 `base_system`을 새로 시작하여 초기 상태를 통일한다. 기존 컨트롤러를 먼저 포함해 종료 순서 1→2→3→4를 마친 뒤, 새 실험 시작 순서 1→3→4→2를 따른다. RViz의 2D Pose Estimate로 위치를 다시 찍으면 이 기준에서 벗어난다.

고정 pose: x=10.832046001482436, y=-0.5021100154908403, yaw=0.3652437316241457rad. hyhall12 global racing-line waypoint 410의 위치와 접선 방향이다. 이전 수동 출발 pose와 다르므로 새 기준 조건으로 구분한다.

설정: `/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/start_pose.yaml`. 설치된 gym_bridge launch는 선택한 맵 디렉터리의 이 파일을 읽어 SIM 기본 sx/sy/stheta보다 우선 적용한다. 파일이 없는 맵은 기존 SIM 기본값을 유지한다. 새 브리지 초기화는 요청 속도·조향과 실제 속도를 0으로 시작한다. 기존 컨트롤러가 살아 있으면 즉시 명령이 전달될 수 있으므로 모든 기존 실행을 종료한 뒤 시작한다. 실행 중 수동 pose 리셋을 고정하는 기능은 아니다.

확인: base_system 시작 로그의 `fixed start pose` 문구 또는 `/bridge`의 sx/sy/stheta 파라미터. 현재 global_limit=0.75 유지. 테스트 6개 통과, f1tenth_gym_ros 빌드 완료, 설치 launch의 실제 맵 pose 해석 확인. 새 pose에서의 주행 성공은 아직 검증하지 않았다.
