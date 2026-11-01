재현성 수정본 repro_v1

기존 ws와 원본 checkout은 보존했다. ws_repro는 random_sampling의 난수 분포 객체를 각 스레드가 소유하도록 수정하고 static 스케줄을 명시한 별도 버전이다.
샘플러 고정입력 50쌍, 합성 상태·기준장·장애물장을 사용하는 solver 고정입력 15쌍이 모두 일치했다. 실제 폐루프 주행 재현성이나 회피 성공을 검증한 것은 아니다.
새 R05는 무장애물·GT·1m/s·예측30개·1000샘플·4스레드·3랩 목표이며 아직 시작하지 않았다. 제어 설정은 R02와 같고 solver 버전만 달라졌다. 기존 결과를 파라미터만 바꾼 비교로 섞지 않는다.

SESSION /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261031T232547_15474_forward_mppi_R05_repro_v1_baseline
Terminal 1:
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
bash manual.sh sim /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261031T232547_15474_forward_mppi_R05_repro_v1_baseline

Terminal 3:
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
bash manual.sh scenario /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261031T232547_15474_forward_mppi_R05_repro_v1_baseline

Terminal 4:
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
bash manual.sh record /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261031T232547_15474_forward_mppi_R05_repro_v1_baseline

Terminal 2:
cd /home/misys/shared_dir/svg_mppi_ros2_20261031
bash manual.sh controller /home/misys/shared_dir/svg_mppi_ros2_20261031/manual_runs/20261031T232547_15474_forward_mppi_R05_repro_v1_baseline

