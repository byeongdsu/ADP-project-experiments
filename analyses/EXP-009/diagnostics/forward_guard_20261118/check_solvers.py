from pathlib import Path
import json,subprocess
r=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');o=r/'diagnostics/forward_guard_20261118';ws=r/'ws_forward_guard';s=(r/'diagnostics/footprint_20261118/probe_svg.cpp').read_text();s=s.replace('cost["raw_occupancy"]=cost["collision_layer"];','cost["raw_occupancy"].setZero();')
for mode,cls,params in [('forward','ForwardMPPI','forward_mppi'),('svg','SVGuidedMPPI','svg_mppi')]:
 text=s.replace('mppi::cpu::SVGuidedMPPI a',f'mppi::cpu::{cls} a').replace('params.svg_mppi',f'params.{params}')
 text=text.replace(' std::cout.precision(17);',f''' cost["raw_occupancy"].setConstant(100.);
 mppi::cpu::{cls} blocked(params.common,params.{params});blocked.set_reference_map(ref);blocked.set_obstacle_map(cost);
 bool rejected=false;try{{blocked.solve(state);}}catch(const std::runtime_error& e){{rejected=std::string(e.what())=="no_collision_free_forward_candidate";}}
 if(!rejected)return 3;
 std::cout<<"{{\\\"blocked_case_rejected\\\":true}}\\n";
 std::cout.precision(17);''')
 p=o/f'probe_{mode}.cpp';p.write_text(text)
 cmd=json.loads((r/f'diagnostics/footprint_20261118/compile_{mode}.json').read_text());cmd=[v.replace(str(r/'ws_footprint'),str(ws)).replace(str(r/'diagnostics/footprint_20261118'),str(o)) for v in cmd];(o/f'compile_{mode}.json').write_text(json.dumps(cmd,indent=2));subprocess.run(cmd,check=True,stdout=(o/f'compile_{mode}.log').open('w'),stderr=subprocess.STDOUT)
 subprocess.run([str(o/f'probe_{mode}'),str(r/'manual_runs/20261118T140204_66033_forward_mppi_B05_berlin_footprint_speed1p5/controller.yaml')],check=True,stdout=(o/f'probe_{mode}.jsonl').open('w'),stderr=subprocess.STDOUT);print(mode,'passed',flush=True)
