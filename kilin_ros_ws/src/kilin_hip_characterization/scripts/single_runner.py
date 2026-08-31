#!/usr/bin/env python3
"""Run exactly one reviewed profile with bag recording and a short command."""
import argparse, signal, subprocess, sys
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument("profile",type=Path);p.add_argument("run_dir",type=Path);p.add_argument("--armed",action="store_true");a=p.parse_args()
    if not a.armed:p.error("refusing to actuate without --armed")
    if a.run_dir.exists():p.error(f"run directory already exists: {a.run_dir}")
    a.run_dir.mkdir(parents=True); target=a.run_dir/"profile.yaml";target.write_text(a.profile.read_text())
    bag=subprocess.Popen(["ros2","bag","record","-o",str(a.run_dir/"bag"),"/motor/state","/motor/command","/power/state","/power/command","/tf","/tf_static"],stdout=(a.run_dir/"bag.log").open("w"),stderr=subprocess.STDOUT)
    try:
        result=subprocess.run(["ros2","run","kilin_hip_characterization","campaign_runner","--ros-args","--params-file",str(target),"-p","armed:=true","-p","command_topic:=/motor/command","-p",f"run_dir:={a.run_dir}"],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (a.run_dir/"launch.log").write_text(result.stdout)
    finally:
        bag.send_signal(signal.SIGINT);bag.wait()
    if result.returncode or "Campaign aborted" in result.stdout or "Campaign refused" in result.stdout:sys.exit(1)
if __name__=="__main__":main()
