#!/usr/bin/env python3
"""Run exactly one reviewed profile with bag recording and a short command."""
import argparse, signal, subprocess, sys
from pathlib import Path
import yaml

DEFAULT_BAG_TOPICS=["/motor/state","/motor/command","/power/state","/power/command","/tf","/tf_static"]

def main():
    p=argparse.ArgumentParser();p.add_argument("profile",type=Path);p.add_argument("run_dir",type=Path);p.add_argument("--armed",action="store_true");a=p.parse_args()
    if not a.armed:p.error("refusing to actuate without --armed")
    if a.run_dir.exists():p.error(f"run directory already exists: {a.run_dir}")
    source=yaml.safe_load(a.profile.read_text())
    params=source["kilin_hip_characterization"]["ros__parameters"]
    bag_topics=params.pop("bag_topics",DEFAULT_BAG_TOPICS)
    a.run_dir.mkdir(parents=True); target=a.run_dir/"profile.yaml";target.write_text(yaml.safe_dump(source,sort_keys=False))
    (a.run_dir/"bag_topics.txt").write_text("\n".join(bag_topics)+"\n")
    print(f"[single] profile={a.profile}")
    print(f"[single] evidence={a.run_dir}")
    print("[single] starting bag recorder")
    bag=subprocess.Popen(["ros2","bag","record","-o",str(a.run_dir/"bag"),*bag_topics],stdout=(a.run_dir/"bag.log").open("w"),stderr=subprocess.STDOUT)
    try:
        command=["ros2","run","kilin_hip_characterization","campaign_runner","--ros-args","--params-file",str(target),"-p","armed:=true","-p","command_topic:=/motor/command","-p",f"run_dir:={a.run_dir}"]
        print("[single] controller started; phase messages follow")
        output=[]
        with (a.run_dir/"launch.log").open("w") as log:
            process=subprocess.Popen(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,bufsize=1)
            for line in process.stdout:
                print(line,end=""); log.write(line); log.flush(); output.append(line)
            result=process.wait()
        output="".join(output)
    finally:
        bag.send_signal(signal.SIGINT);bag.wait()
    print(f"[single] controller exit={result}; bag closing")
    if result or "Campaign aborted" in output or "Campaign refused" in output:sys.exit(1)
    print(f"[single] complete: {a.run_dir}")
if __name__=="__main__":main()
