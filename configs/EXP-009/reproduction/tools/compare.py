"""Build one functional-observation report from explicitly selected completed runs."""
import csv,json,sys,math
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
root=Path(__file__).resolve().parents[1];runs=[Path(s).resolve() for s in sys.argv[1:]]
race=Path('/home/misys/forza_ws/race_stack');mapdir=race/'stack_master/maps/hyhall12'
m=yaml.safe_load((mapdir/'hyhall12_orig.yaml').read_text());im=Image.open(mapdir/'hyhall12_orig.png');ox,oy,_=m['origin'];r=m['resolution'];wp=json.loads((mapdir/'global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'];points=np.array([(p['x_m'],p['y_m']) for p in wp])
rows=[];selected={};fig,axs=plt.subplots(1,2,figsize=(17,7))
for ax,obstacles in zip(axs,[0,2]):
 ax.imshow(im,extent=[ox,ox+im.width*r,oy,oy+im.height*r],origin='upper',cmap='gray')
 ax.plot(points[:,0],points[:,1],'--',color='gray',label='reference')
 ax.set_title(f'{obstacles} obstacles, GT, 1.0 m/s');ax.set_aspect('equal');ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
for p in runs:
 s=json.loads((p/'summary.json').read_text());assert s['reason']=='lap_target' and s['error'] is None
 selected[f"{s['mode']}_obs{s['obstacles']}"]=p.name
 scorer=json.loads(next(p.glob('race_result_*.json')).read_text())
 row={'run':p.name,'mode':s['mode'],'obstacles':s['obstacles'],'laps':s['laps_completed'],'collisions':len(s['collisions']),'stop_wall_s':s['stopped_wall_s'],'observation_wall_s':s['observed_wall_s'],'credited_lap_s':scorer['laps'][0]['time'],'solver_mean_ms':s['solver_ms']['mean'],'solver_p95_ms':s['solver_ms']['p95'],'cycle_mean_ms':s['cycle_ms']['mean'],'tracking_nearest_waypoint_mean_m':s['gt_nearest_waypoint_distance_m']['mean'],'tracking_nearest_waypoint_max_m':s['gt_nearest_waypoint_distance_m']['max']};rows.append(row)
 a=np.loadtxt(p/'trajectory.csv',delimiter=',',skiprows=1);ax=axs[0 if s['obstacles']==0 else 1];ax.plot(a[:,1],a[:,2],label=s['mode'],linewidth=1.5)
 if s['obstacles']==2 and s['mode']=='forward_mppi':
  layout=json.loads((p/'obstacle_layouts.json').read_text())[0]['poses']
  for ob in layout:
   theta=2*math.atan2(ob['qz'],ob['qw']);rot=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]);corners=np.array([[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]])@rot.T+[ob['x'],ob['y']];ax.add_patch(Polygon(corners,facecolor='red',edgecolor='darkred'))
for ax in axs:ax.legend()
fig.tight_layout();fig.savefig(root/'comparison.png',dpi=180);plt.close(fig)
layouts=[]
for p in runs:
 s=json.loads((p/'summary.json').read_text())
 if s['obstacles']==2:layouts.append(json.loads((p/'obstacle_layouts.json').read_text())[0]['poses'])
assert len(layouts)==2 and layouts[0]==layouts[1], 'Actual initial obstacle layouts differ'
with (root/'comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(root/'comparison.json').write_text(json.dumps({'runs':rows,'selected_runs':selected,'actual_initial_obstacle_layouts_equal':True,'initial_obstacle_layout':layouts[0]},indent=2))
text='''# ROS 2 MPPI / SVG-MPPI 시뮬레이션 기능 확인 — 2026-10-31

기존 race_stack 변경 없이 별도 ROS 2 패키지를 빌드하고, 기존 HYHALL12 Gym에서 두 알고리즘의 실제 움직임을 확인했다.
위치 입력은 GT, 명령 속도 1.0m/s, 샘플 1000개, OpenMP 4개 스레드, 예측 상태 15개/간격 0.05초다.
원본 solver 10개 파일은 지정 커밋과 바이트 해시가 일치한다. ROS 입출력과 비용장 생성은 새 어댑터다.

| 모드 | 장애물 | 평가기 완료 랩 | 관찰 충돌 | 인정 랩(s) | solver 평균 / p95(ms) | 최근접 waypoint 거리 평균 / 최대(m) |
|---|---:|---:|---:|---:|---:|---:|
'''
for row in rows:text+=f"| {row['mode']} | {row['obstacles']} | {row['laps']} | {row['collisions']} | {row['credited_lap_s']:.3f} | {row['solver_mean_ms']:.2f} / {row['solver_p95_ms']:.2f} | {row['tracking_nearest_waypoint_mean_m']:.3f} / {row['tracking_nearest_waypoint_max_m']:.3f} |\n"
text+='''
![GT trajectories](comparison.png)

각 조건은 한 번의 관찰이다. 이 조건에서 무충돌 주행을 확인했으나 일반적인 성공률·강건성·알고리즘 우열을 결론내리지 않는다.
특히 기본 MPPI의 경로 이탈이 커서 추종 품질 개선 과제가 남는다. 샘플 수가 같아도 SVG의 guide/gradient 계산으로 계산량은 다르다.
거리 통계는 waypoint 점까지의 거리이며 polyline 오차가 아니다. lap 집계와 출발부터의 전체 관찰 시간도 다르다.
정지 시간에는 출발 가속 구간이 포함되며 15초 지속 정지는 관찰되지 않았다.
계산 시간은 solver.solve 호출만 측정했고 callback 전체 시간은 각 summary.json의 cycle_ms에 따로 저장했다.

장애물 조건은 기존 race_manager seed 2026의 2개 배치를 사용했다. 두 모드의 실제 최초 배치 XY와 quaternion은 일치했다.
최종 비교용 장애물 실행은 실제 PoseArray 수신 후 출발하도록 했다. 목표 랩 도달 직후 새 배치를 수신할 수 있으나 그 배치 통과를 검증한 것은 아니다.
원본 EXP-005 고정 배치 기능을 복원하거나 새 고정 배치 기능을 추가하지 않았다.

기록: 원본 rosbag, 실행 YAML, 명령, 노드별 로그, GT 궤적 CSV/PNG, 실제 장애물 배치, 계산 시간 CSV, 기존 race_manager 결과 JSON.
초기 CLI/노드명 오류 및 중단한 탐색 실행은 보존했고 이 표에 포함하지 않았다. 한 중단 실행의 rclpy SIGINT 정리 오류도 로그로 보존했으며 실행기를 수정했다.
시뮬레이터의 기존 SIGINT 종료 시 shutdown 중복 traceback은 정상 주행 중의 solver 오류와 구분했다.
GUI는 X11 인증 문제로 열지 못했고 headless 실행 및 저장된 그림으로 관찰했다.

실행 방법과 구현 차이는 [README.ko.md](README.ko.md), 선택된 실행 목록 및 수치는 [comparison.json](comparison.json), [comparison.csv](comparison.csv).
기존 ROS 2 작업공간·설정·EXP-001~007 결과와 원본 MPPI checkout은 수정하지 않았다. PF·실차·PPO는 포함하지 않았다.
'''
(root/'REPORT.ko.md').write_text(text)
print(json.dumps(rows,indent=2))
