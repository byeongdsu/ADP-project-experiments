from pathlib import Path
import subprocess,json
root=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');out=root/'diagnostics/footprint_20261118';ws=root/'ws_footprint'
s=(root/'diagnostics/berlin_integration_20261118/check_stationary.py').read_text().replace("glob('*B01_berlin_speed1p0')","glob('*B05_berlin_footprint_speed1p5')").replace("'-p','reference_speed:=1.0'","'-p','reference_speed:='+str(s['reference_speed'])").replace("{'collision_layer','clearance_cost'}","{'raw_occupancy','collision_layer','clearance_cost'}").replace('np.allclose(speeds,1.)',"np.allclose(speeds,s['reference_speed'])")
(out/'check_stationary.py').write_text(s)
s=(root/'diagnostics/svg_repro_berlin_20261118/solver_probe.cpp').read_text().replace('#include "mppi_controller/stein_variational_guided_mppi.hpp"','#include "mppi_controller/stein_variational_guided_mppi.hpp"\n#include "mppi_controller/forward_mppi.hpp"')
s=s.replace('cost({"collision_layer"})','cost({"collision_layer","raw_occupancy"})').replace('auto state=mppi::cpu::State::Zero().eval();','cost["raw_occupancy"]=cost["collision_layer"];\n auto state=mppi::cpu::State::Zero().eval();')
for mode,cls in [('forward','ForwardMPPI'),('svg','SVGuidedMPPI')]:
 text=s.replace('mppi::cpu::SVGuidedMPPI a',f'mppi::cpu::{cls} a')
 if mode=='forward':text=text.replace('params.svg_mppi','params.forward_mppi')
 p=out/f'probe_{mode}.cpp';p.write_text(text)
 cmd=['g++','-I'+str(ws/'src/svg_mppi_ros2/src'),'-std=c++17','-O3','-DNDEBUG','-fopenmp','-I'+str(ws/'src/svg_mppi_ros2/include'),'-I/usr/include/eigen3','-I/opt/ros/humble/include/grid_map_core','-DEIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp"','-DEIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"',str(p),str(ws/'build/svg_mppi_ros2/libsvg_mppi_core.a'),'-o',str(out/f'probe_{mode}'),'/opt/ros/humble/lib/libgrid_map_core.a','-lyaml-cpp']
 (out/f'compile_{mode}.json').write_text(json.dumps(cmd,indent=2))
 subprocess.run(cmd,check=True,stdout=(out/f'compile_{mode}.log').open('w'),stderr=subprocess.STDOUT)
 cfg=root/'manual_runs/20261118T130823_42578_forward_mppi_B03_berlin_speed1p5/controller.yaml'
 subprocess.run([str(out/f'probe_{mode}'),str(cfg)],check=True,stdout=(out/f'probe_{mode}.jsonl').open('w'),stderr=subprocess.STDOUT)
 print(mode,'passed',flush=True)
