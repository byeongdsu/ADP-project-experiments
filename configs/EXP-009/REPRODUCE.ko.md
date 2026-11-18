# EXP-009 재현 자료 — 2026-11-18

실제 실행 루트 `/home/misys/shared_dir/svg_mppi_ros2_20261031`, ROS2 Humble 기존 컨테이너 기준. 이 폴더는 실험 기록용 사본이며 checkout만으로 바로 실행 가능한 전체 simulator 배포가 아니다. 원래 race_stack 작업 트리와 install이 별도로 필요하다. 원본 solver `f4fb7b74f12d4568790213aab26dd6f0538d1925`는 ROS1 Noetic이고, 이 자료는 별도 ROS2 어댑터·수정본이다. 라이선스는 [LICENSE.upstream](reproduction/LICENSE.upstream).

## 소스 사본

| workspace | 이전 버전 대비 변경 |
|---|---|
| ws_consistent | 기존 ws_clearance에 전역/로컬 예측 초기 조향 일치 |
| ws_svg_repro | SVG guide gradient 공유 행렬 쓰기를 샘플별 계산 + 고정 순서 합산 |
| ws_footprint | 미래 x/y/yaw 직사각형 대 raw 점유 셀 검사, 기하 거리 soft 비용 |
| ws_forward_guard | 전방 진행, 후보 제외, 평균 대체, 최종 경로 검증, latch 정지 |

`ws`, `ws_repro`, `ws_speed`, `ws_clearance`의 이전 스냅샷은 [EXP-008](../EXP-008/reproduction/)에 있다. 이번 `reproduction/tools`, `manual.sh`는11/18 최신 사본으로 당시 run별 `runner.py`와 다를 수 있다. 생성한 기존 세션을 재사용하지 않는다. 파일 경로가 고정된 부분은 별도 사본에서 검토한다.

## 실행 환경·지도

Ubuntu22.04, ROS2 Humble, 기존 race_stack/grid_map/ackermann/nav/sensor/std/geometry msgs, Eigen, OpenMP, yaml-cpp, NumPy/SciPy/Matplotlib, ROS2 bag/SQLite 필요. Gym backend는 기존 설치된 f110_gym + gymnasium이다. Docker 신규 생성/이미지 정리/컨테이너 삭제는 하지 않았다. `race_support`는 사용한 일부 소스·차량설정 사본이며 전체 race_stack 배포가 아니다.

`maps/berlin`은 실제 사용한 지도/경로/속도 파일과 사용자 승인 출발 설정이다. 원본 네 파일은 저장소 루트의 사용자 커밋 a34fb4b에도 있다. speed_scaling은 기록만 하며 현재 고정 목표 속도에는 적용하지 않는다. 각 실행 session.json의 map_asset_sha256와 실제 파일을 대조한다.

## 별도 빌드 예시

현재 로컬 환경에서 새 작업을 시작할 때의 참고 명령이며, 이 게시 작업 중 새 주행을 실행하지 않았다. 기존 src/install을 덮어쓰지 않는다.

```bash
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
colcon --log-base ws_forward_guard/log build --base-paths ws_forward_guard/src --build-base ws_forward_guard/build --install-base ws_forward_guard/install --parallel-workers 1
```

**현재 B07 guard 버전에는 계산 회귀가 남아 있다.** core O3 / node 비최적화 공유 검사 경로를 먼저 검토한다. 이 명령은 수정 완료·40Hz 달성·안전 보장을 뜻하지 않는다.

## 신규 수동 세션 예시

```bash
bash manual.sh prepare svg_mppi --solver-version forward_guard_v1 --map-name berlin --config tuning/B06_berlin_footprint_svg_speed1p5.yaml --obstacles 4 --seed 2026 --laps 7 --duration 600 --domain 76 --label new_guard_review
```

위 tuning 경로는 실제 로컬 파일이다. Git 사본에서는 B07의 `controller.yaml`을 별도 tuning 파일로 복사한 뒤 지정한다. prepare가 생성하는 TERMINALS.txt의 **새 절대 경로**를 사용한다. 순서1 sim→3 scenario→4 record→2 controller. controller 종료 후1→3→4를Ctrl+C, recorder prompt까지 기다린다. 다른 PP/TLN/teleop의 drive publisher와 동시 실행 금지.

guard는 후보가 없거나 반환 경로가 부적합하면 목표속도0을 반복하고 자동 재출발하지 않는다. 속도.05미만1초 유지 후 runner가 guard_stop으로 종료한다. 정지를 회피 성공으로 세지 않는다. 노이즈·시간 지연·비가시 영역 때문에 충돌0을 보장하는 검증이 아니다. 원본 footprint/consistent 실행은 자동 위험 감속이 없는 고정 속도 구현이다.

## 원본 데이터와 검증

[로컬 bag 목록](../../analyses/EXP-009/local_bag_inventory.json)과 [실행별 기록](../../runs/EXP-009/)을 참조한다. 원본 bag은 로컬에만 있고, Git에는 metadata 요약 사본과 분석 결과가 있다. ros_logs/lock/compiled probes/빌드트리/실행 binary/대형 영상은 제외했다. 가공 자료 파일 해시는 [artifact_manifest.json](../../analyses/EXP-009/artifact_manifest.json)에 있다. source별 manifest/diff는 진단 폴더에 포함했다.

snapshot을 기존 작업공간에 자동 덮어쓰지 않는다. 원래 run의 조건·소스 해시·runner와 최신 도구 차이를 확인하고 새 경로/새 세션으로 재현한다.
