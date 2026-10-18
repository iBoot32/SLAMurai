# Architecture (set by the user, keep it)

This project is a small library that lets an agent drive a Nav2 robot and read data back from it. It is agent first.
The user talks to an agent about a problem. The agent proposes experiments, the user agrees, the agent writes a script
that uses the library, runs it on the robot, analyses the data and reports. Optimization is one possible outcome.
Noting problems for discussion is another.

## Layers
- `robotlab/` PC side library. `robot.py` is the connection, `trial.py` drives and records, `metrics.py` and `objective.py` score,
  `lab.py` holds the common work (run a route with parameter changes, sweep, compare, optimize, logging).
- `robotlab/profiles/*.yaml` everything robot-specific: topics, types, frames, how goals are sent, how Nav2 restarts, where the
  safety status comes from, where the Nav2 parameters are. A new robot means a new profile.
- `recipes/` example experiment scripts the agent copies and edits.
- `robot_side/` runs on the robot: the safety gate and its core, the Nav2 restart helper, human tools (commission, approve, arm), config.
  Off unless the robot launches with `safety:=true`. The library only knows it through the profile.

## Rules
1. The library is glue plus common helpers. It sends goals, reads data, sets Nav2 parameters, records logs, and offers sweep,
   compare and optimize as plain calls. It never decides what to test.
2. No robot facts in code. They live in the profile.
3. No speed, range or pose limits in the library. The agent's script picks values. The safety layer enforces limits.
4. The library never publishes velocity.
5. Tuning is live, on the real robot, in the real environment. No simulation.
6. No test suite in this repo. Check changes with a recipe against the robot.
7. Minimal comments and docs. Short code.
8. Keep the original robot code as is. This repo adds only `robotlab/`, `recipes/` and `robot_side/`, plus the `safety` launch argument and the `cmd_topic` parameter.
9. Ask the user before restructuring or deleting.

## Assumptions that stay
ROS 2 Dashing and Nav2 (parameters apply only at node start, so a parameter change means a Nav2 restart).
The PC reaches the robot through rosbridge.
