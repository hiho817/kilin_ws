#!/usr/bin/env python3
"""Run sequential unit tests from one master YAML; stop on the first fault."""
import argparse, signal, subprocess, sys
from pathlib import Path
import yaml

def main():
    p=argparse.ArgumentParser();p.add_argument("master",type=Path);p.add_argument("run_root",type=Path);p.add_argument("--armed",action="store_true");a=p.parse_args()
    if not a.armed: p.error("refusing to actuate without --armed")
    data=yaml.safe_load(a.master.read_text()); spec=data["kilin_hip_batch"]
    defaults=spec["defaults"]; tests=spec["tests"]; a.run_root.mkdir(parents=True,exist_ok=False)
    for index,test in enumerate(tests,1):
        name=test["name"]; params=dict(defaults);params.update(test.get("parameters",{})); params.pop("name",None)
        run=a.run_root/f"{index:03d}_{name}";run.mkdir(); profile=run/"resolved_profile.yaml"
        print(f"[batch] {index}/{len(tests)} starting {name}")
        params.update({"armed":True,"command_topic":"/motor/command","run_dir":str(run)})
        profile.write_text(yaml.safe_dump({"kilin_hip_characterization":{"ros__parameters":params}},sort_keys=False))
        bag=subprocess.Popen(["ros2","bag","record","-o",str(run/"bag"),"/motor/state","/motor/command","/power/state","/power/command","/tf","/tf_static"],stdout=(run/"bag.log").open("w"),stderr=subprocess.STDOUT)
        try:
            output=[]
            with (run/"launch.log").open("w") as log:
                process=subprocess.Popen(["ros2","run","kilin_hip_characterization","campaign_runner","--ros-args","--params-file",str(profile)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,bufsize=1)
                for line in process.stdout:
                    print(f"[{name}] {line}",end=""); log.write(line); log.flush(); output.append(line)
                result=process.wait()
            output="".join(output)
        finally:
            bag.send_signal(signal.SIGINT);bag.wait()
        if result or "Campaign aborted" in output or "Campaign refused" in output:
            print(f"Batch stopped at {name}; see {run}",file=sys.stderr);sys.exit(1)
    print(f"Batch complete: {a.run_root}")
if __name__=="__main__":main()
