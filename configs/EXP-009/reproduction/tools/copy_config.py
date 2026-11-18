#!/usr/bin/env python3
"""Create a new config without replacing previous configs or experiment snapshots."""
import argparse,yaml
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--mode',choices=['forward_mppi','svg_mppi']);p.add_argument('--set',action='append',default=[]);a=p.parse_args();cfg=yaml.safe_load(a.source.read_text());changes=[]
if a.mode:changes.append(('mpc_mode',cfg['mpc_mode'],a.mode));cfg['mpc_mode']=a.mode
for entry in a.set:
 key,value=entry.split('=',1);parts=key.replace('/','.').split('.');node=cfg
 for part in parts[:-1]:node=node[part]
 old=node[parts[-1]];new=yaml.safe_load(value)
 if type(new)!=type(old):raise ValueError(f'{key} must preserve value type {type(old).__name__}')
 node[parts[-1]]=new;changes.append((key,old,new))
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('x') as f:f.write('# Derived from '+str(a.source.resolve())+'\n'+yaml.safe_dump(cfg,sort_keys=False))
for key,before,after in changes:print(f'{key}: {before!r} -> {after!r}')
print(a.output.resolve())
