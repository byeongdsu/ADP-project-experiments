# 2026-11-18 전방 진행·최종 궤적 검증 버전

사용자 B06 SVG 역주행 분석 후 공통 전방 진행 조건/최종 궤적 검증/불가시 감속·정지 구현 승인. 시뮬레이션만. 기존 ws_footprint를 새 ws_forward_guard에 복사, 기존 binary/source/config/bag/race_stack 변경은 보존. selector forward_guard_v1. 원본 ROS1 commit f4fb7b74f12d4568790213aab26dd6f0538d1925에 대한 별도 ROS2 수정본이며 upstream 자체 기능이라고 표현하지 않는다.

## 동작 변경 (양 알고리즘 공통)

1. 모든 미래 상태에 대해 최근접 참조각 필드와 차체 yaw의 cos차이>0 요구(반대방향 거부). 각 예측 이동의 참조 접선방향 투영이 -1mm미만이면 거부. 움직이는 경로에 양의 누적 진행 요구. 기존 위치·각도 비용과 별도로 투영 진행거리당 -2 비용을 추가(2/m는 이번 고정 설계값, 튜닝최적값아님). 전역 arc index 보상이나 학습 모델 아님.
2. 2.2초 전체 예측 구간에서 차체-raw 점유 격자 충돌 또는 방향 부적합이 있으면 state cost에1e9 sentinel. collision component 자체는 그대로 유지하여 collision_rate가방향위반율로바뀌지않음. Forward/SVG의 최종 중요도 가중치에서 sentinel후보는 정확히0. 모두 부적합하면 명시적 예외. SVG guide gradient에는 큰 비용이 반영되며, 최종 후보 가중치에서 hard exclusion. 후보단계의 방향 판정은 이산예측점 기준.
3. 가중평균 제어의 재예측이 이산 기준 부적합하면 최대 가중치의 유효 후보 제어로 대체. 이것은 수정본의 추가 동작이고 baseline MPPI/SVG와 구분.
4. 발행 직전 실제 반환 경로를 다시 검증. 예측점 사이도 최대2cm/2도 간격으로 보간, 회전 차체/방향/맵 경계 확인. 검증 전에 양의 속도 구동 명령을 보내지 않음. best_path는 검증 대상 제안 경로를 기록하므로 guard_stop시 실행된 경로라는 뜻은 아님.
5. 후보 없음/최종 검증실패/solver 예외시 guard_stop 상태, 목표속도0/이전 조향 유지, 반복 발행. 실제 감속은 Gym 속도제어기/동역학의 응답. 정지는 latch하여 장애물 소멸/enable true에도 자동 재출발하지 않음. 재실행은 새세션으로. 고정목표1.5는 통과한 경로에만 발행, 실제속도/예측모델의 상시 가변 속도 정책이나 PPO는 아직 없음. 재계획으로 낮은 속도의 회피해를 찾는 기능도 이번범위아님.
6. runner는 guard_stop후 GT절대속도.05미만이1초 유지되면 reason guard_stop으로 종료, final_controller_status에 사유보존. 기존 first_collision 판정 우선. 종료 후 별도sole stop publisher20회 송신은 유지. runner 변경 전 파일/diff보존.

## 보존·비교

다음 준비 세션 manual_runs/20261118T142140_80314_svg_mppi_B07_berlin_svg_forward_guard_speed1p5.
B06와 controller.yaml/simulator.yaml 바이트 동일, Berlin 지도와 경로/시작 위치 해시 동일. SVG/GT/명목1.5m/s/4장애물/seed2026/45states,dt.05/7lap/600s. 변경점은 공통 비용/후보 제외/평균대체/최종검증/정지 정책. 이것은 한 파라미터 실험이 아닌 공통 실행 정책 수정 검증이다. 원본baselineB05/B06와 구분해 기록. 동일 seed의 랩별 실제 장애물 배치도 비교 필요.

## 검증 완료

- colcon build 성공 (ws_forward_guard/build.log).
- test_forward_guard: 정방향 직선/곡선, 역방향 yaw, 차체전방과다른후방이동, NaN, reference 밖, sparse endpoint사이충돌, 가중치0 및 전후보부적합 예외 확인.
- 실제 solver 고정입력 thread4: forward/SVG 각15쌍 control 불일치0/nonfinite0. 빈 raw map에서통과, raw전점유에서 명시적 no_collision_free_forward_candidate 확인. 이전 fixture의 legacy collision layer는무시하고raw빈맵을쓴테스트라는점유의(기존probe의fixture 문자열보다 source참조).
- synthetic ROS domain79: 각모드에정방향GT/0.5m전방가로막는raw벽입력, initial alignment정상, guard_stop, 비영속도명령0. 맵을비우고enable반복해도latch유지. 물리차량/실제시뮬레이터는이테스트에서시작하지않음. guard_node_test.json.
- Berlin 실제입력 정지체크domain78 passed:맵600x600/reference489/GTstart일치/4obstacles/rawlayer/ready_disabled/nonzero0. enabled주행아님. stationary_check.json.
- 오프라인 실제Gym 동일차량/지도 수치검사:명목1.5에서0목표,조향0/±.4189 유지. |v|<.05까지 .22초/이동약.126m,이후잔여포함약.128m. 통신/계산지연없는유한조건테스트,충돌없는제동거리보장이나실시간안전보장이아님. braking_test.json. 초기스크립트가옛gym API를사용해실패,실제bridge와같은gymnasium/poses키워드로수정후통과.

## 아직 확인하지 않은 것

실제 B07 주행/역주행억제/완주/새랩타임은 미측정. 현재 scan 가시영역/지연/동역학추정오차/이산보간한계가있어충돌0보장안함. 전체horizon hard collision 검사는 보수적이어서 기존보다 일찍 정지할 수 있고, 멈춤은 회피 성공아님. 중단/정지/충돌/완주를분리해기록. 안전제약변경이샘플탐색부족을해결했다고표현금지. 이후통과하는후보가부족하면유효후보수/입력/제약/노이즈를진단한뒤낮은속도재계획을별도실험. RL학습은미시작.

원본보존 manifest/diff: source_manifest.json,variant.diff,manual_session.diff,run_sim.diff. 실행1->3->4->2,종료2자동후1->3->4,recorder prompt대기.
