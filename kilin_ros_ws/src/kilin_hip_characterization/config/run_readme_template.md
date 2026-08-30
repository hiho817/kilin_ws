# Hip characterization run record

Use one row per immutable parameter cell. The three repetitions belong to the same run folder only when the profile and robot/setup conditions stay unchanged.

| Run | Group | Speed | PID (kp/ki/kd) | Hip FF | Wheel mode and value | Profile copy | Bag | Result / notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `hip_front_rest_kp350_ff0_speed2_set01` | A+B front | Needs verification | 350/0/5 | 0 Nm | rest | `profile.yaml` | `bag/` | _fill after run_ |

Before arming: confirm no competing `/motor/command` authority, fresh `/motor/state`, correct post-fix angle convention, current wheel position captured for position mode, free disk space, and recorder running.

After every run: state whether it completed, aborted, or was operator-stopped; preserve terminal output; note setup changes, ambient/thermal state, and any direction-sign observation.
