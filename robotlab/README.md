# robotlab

A small library an agent uses to run experiments on a Nav2 robot from a PC. You describe a problem in chat, the agent
writes a script on top of this library, runs it on the robot, analyses the data and reports.

```
agent script (recipes/)  ->  lab.py  ->  robot.py  --rosbridge/Wi-Fi-->  robot
                              sweep       goals, streams, restart            safety gate (robot_side/safety)
                              compare     all names from profiles/*.yaml     Nav2 restart (robot_side/tools/nav2_manager.py)
                              optimize                                       firmware limits (control/)
                              logs
```

- `robot.py` connects and exposes `pose()`, `velocity()`, `cmd_in()`, `cmd_out()`, `plan()`, `status()`, `send_goal()`, `cancel()`,
  `sleep()` (sends the heartbeat), `restart_nav2(params)`.
- `trial.py` drives a goal or a route and records arrays. `metrics.py` scores a trial. `objective.py` turns metrics into a loss.
- `lab.py` runs a route with parameter changes (restarts Nav2 when they change), `sweep`, `compare`, `optimize`, and logs every
  trial to `work/lab.jsonl` and `work/trials/*.npz`.
- The library never publishes a velocity and holds no speed, range or pose limits. The safety layer on the robot enforces those.

## A new robot

Copy `profiles/slamurai.yaml` and fill in: host, goal topic and type, the streams (odometry, localization pose, `/cmd_vel`, plan,
log, tf, safety status), frame names, how Nav2 is restarted, and where the Nav2 parameter file is. Pass the profile to
`connect()`. If the robot has no safety status or restart helper, leave those sections out.

## Install

```
pip install roslibpy numpy scipy pyyaml        # PC
pip install pytest websockets                   # tests only
```

## Tests

`cd tests && python -m pytest -q`  (the firmware tests need a C++ compiler: g++, clang++ or `pip install ziglang`)
