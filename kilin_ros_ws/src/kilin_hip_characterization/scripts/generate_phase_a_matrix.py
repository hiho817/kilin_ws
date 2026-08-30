#!/usr/bin/env python3
"""Generate immutable, disarmed ROS profiles from one reviewed campaign YAML."""
import argparse, itertools
from pathlib import Path
import yaml

GROUPS={"front":["A","B"],"rear":["C","D"],"left":["A","C"],"right":["B","D"]}
def need(x,key):
    if key not in x: raise ValueError(f"missing required key: {key}")
    return x[key]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument("config",type=Path);p.add_argument("out_dir",type=Path);p.add_argument("--stage",required=True);a=p.parse_args()
 data=yaml.safe_load(a.config.read_text()); campaign=data.get("campaign",{})
 defaults=dict(need(campaign,"defaults")); stages=need(campaign,"stages")
 if a.stage not in stages:p.error(f"unknown stage {a.stage!r}; choose: {', '.join(stages)}")
 stage=stages[a.stage]; m=need(stage,"matrix"); groups=need(m,"groups"); speeds=need(m,"hip_speed_rad_s"); pids=need(m,"pid_candidates"); ffs=need(m,"hip_ff_direct"); wheels=need(m,"wheel_modes")
 if not set(groups)<=set(GROUPS):p.error("unknown group")
 if defaults.get("repetitions",0)<3:p.error("repetitions must be at least 3")
 a.out_dir.mkdir(parents=True,exist_ok=False);rows=[]
 for i,(g,s,gain,ff,w) in enumerate(itertools.product(groups,speeds,pids,ffs,wheels),1):
  if len(gain)!=3:p.error("PID candidate must be kp,ki,kd")
  x=dict(defaults);x.update(stage.get("overrides",{}));x.update({"active_modules":GROUPS[g],"hip_speed_rad_s":s,"kp":gain[0],"ki":gain[1],"kd":gain[2],"wheel_mode":w,"hip_ff_outward_direct":ff,"hip_ff_inward_direct":ff})
  if w!="torque":x.update({"wheel_torque_outward_nm":0.0,"wheel_torque_inward_nm":0.0})
  name=f"cell{i:03d}_{g}_{w}_hip{s:g}rad_s_kp{gain[0]:g}_ki{gain[1]:g}_kd{gain[2]:g}_ff{ff:g}"
  text=f"# Generated from {a.config.name}, stage {a.stage}. Do not edit after execution.\n"+yaml.safe_dump({"kilin_hip_characterization":{"ros__parameters":x}},sort_keys=False)
  (a.out_dir/f"{name}.yaml").write_text(text);rows.append((name,g,s,"/".join(f"{v:g}" for v in gain),ff,w))
 lines=[f"# Generated cells: {a.stage}","",f"Source: `{a.config}`",f"Strategy: `{defaults['strategy_name']}` v`{defaults['strategy_version']}`","","| Cell | Group | speed rad/s | PID | FF | Wheel | Result |","| --- | --- | ---: | --- | ---: | --- | --- |"]+[f"| `{n}` | {g} | {s:g} | {q} | {f:g} | {w} | _fill after run_ |" for n,g,s,q,f,w in rows]
 (a.out_dir/"README.md").write_text("\n".join(lines)+"\n");print(f"Generated {len(rows)} cells for {a.stage} in {a.out_dir}")
if __name__=="__main__":main()
