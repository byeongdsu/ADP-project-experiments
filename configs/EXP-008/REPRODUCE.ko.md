# MPPI 자료와 재현 안내

기록 기준 실행 루트는 `/home/misys/shared_dir/svg_mppi_ros2_20261031`이다. 이 폴더는 git 저장소가 아니었으므로 실험 기록 저장소에 소스·설정 사본을 별도로 보존했다. 기존 race_stack 작업 트리는 commit/reset하지 않았다.

## 원본과 변형

원본 https://github.com/kohonda/proj-svg_mppi 의 실제 checkout은 `f4fb7b74f12d4568790213aab26dd6f0538d1925`이며 변경하지 않았다. ROS1 Noetic 공식 구현이고 이번 실행은 ROS2 Humble 어댑터이므로 구분한다. upstream MIT 라이선스는 `reproduction/LICENSE.upstream`에 있다.

`reproduction/`의 `ws`, `ws_repro`, `ws_speed`, `ws_clearance`는 서로 다른 소스 변형이다. 사용 버전과 executable/config SHA는 각 run의 session/conditions JSON을 참조한다. 과거 binary 자체는 게시하지 않으며 재빌드 hash가 동일하다고 보장하지 않는다. 빌드환경 차이를 기록한다.

## 실행 환경

Ubuntu22.04/ROS2 Humble, 기존 race_stack install, grid_map/ackermann/nav/sensor/std/geometry msgs, Eigen, OpenMP, yaml-cpp, NumPy/SciPy/Matplotlib, ROS2 bag/SQLite 의존성이 필요하다. 현재 환경 스냅샷은 `analyses/EXP-008/environment.json`이다. Docker/Noetic 신규 이미지나 catkin 설치를 완료했다는 의미가 아니다.

archive의 race_support는 관련 의존 파일 일부이며 완전한 simulator/맵 배포가 아니다. 기존 HYHALL12와 race_stack 설치가 필요하다. 설치 경로는 도구 내부에 명시돼 있으므로 다른 머신에서는 원본을 덮어쓰지 말고 별도 복사본에서 경로를 검토한다. 이 git 저장소 checkout에서 manual.sh를 그대로 실행하면 필요한 ws/install이 없어 실패할 수 있다.

## 현재 로컬 환경에서 별도 빌드 예시

새 실행을 시작하라는 지시가 아니라 재현 명령 기록이다. 기존 컨테이너에서:

```bash
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
colcon --log-base ws_clearance/log build --base-paths ws_clearance/src --build-base ws_clearance/build --install-base ws_clearance/install --parallel-workers 1
```

기존 run은 수정·재사용하지 않고 반드시 prepare로 새 경로를 만든다.

```bash
python3 tools/manual_session.py prepare forward_mppi --solver-version clearance_v1 --config tuning/R24_h45_speed3p0_clearance.yaml --obstacles 4 --seed 2026 --laps 7 --duration 240 --domain 76 --label new_review
```

생성된 TERMINALS.txt의 절대경로를 사용한다. 실행 순서1 sim→3 scenario→4 record→2 controller. 7랩/첫충돌/15초정지/240초 중 먼저 발생하면 종료한다. controller 종료정지와 실제속도를 확인하고1→3→4를Ctrl+C로 종료한다. record 저장완료까지 기다린다. 다른 PP/TLN 제어기와 동시 drive 발행하지 않는다. `clearance_v1`은 별도 비용 입력 스크립트를 직접 실행하며 원본 ros2 run 입력을 잘못 연결하면 필요한 layer가 없어 대기한다.

RViz 설정 사본은 `visual/mppi.rviz`이며 호스트 X 접근이 필요하다. `host-display.txt`는 머신 전용이므로 보관에서 제외했다. 기존 컨테이너 삭제/image prune 명령은 사용하지 않는다.

## 오프라인 분석

```bash
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
python3 tools/analyze_speed3_failures.py
```

이 분석은 로컬 원본 bags를 SQLite read-only로 읽고 diagnostics에 JSON/PNG를 생성한다. 내부에 기록된 특정 run 이름과 로컬 경로가 필요하다. 데이터가 없는 외부 checkout에서 그대로 실행된다고 보장하지 않는다. 원본 bag 위치·크기는 `analyses/EXP-008/local_bag_inventory.json`, Git에 포함한 파일 해시는 `analyses/EXP-008/artifact_manifest.json`에 있다.

현재 스크립트 사본과 run별 당시 runner.py는 다를 수 있다. 역사적 재현은 당시 runner/조건/소스 해시를 우선한다. 큰bag·영상·빌드산출물·실행binary는 게시하지 않는다.
