# What differs from `Desktop/SLAMurai` (the original is untouched; the copy omits `.git`)

New:
* `robot_side/` (safety gate and core, robot-side tools, config), `robotlab/` (PC library), `recipes/`, `tests/`
* `control/motor_controller_xdrive_safe/` - `.ino` + `safety_limits.h` (the original `motor_controller_xdrive` is kept)
* `autonav/nav/launch/slamurai_tuning_launch.py` - the original launch without Nav2, plus the safety gate (started disarmed)
* `autonav/nav/launch/nav2_only_launch.py` - Nav2 alone with `params:=`/`map:=`/`bt:=` (restarted per candidate)

Changed in the copy:
* `autonav/nav/src/main.py` (serial bridge): subscribes `/cmd_vel_safe` instead of `/cmd_vel`; re-sends the last command at 20 Hz;
  zeros if silent 0.3 s; parses the optional 7th odometry field and publishes it on `/fw_flags`; drains the serial buffer each tick.
* `autonav/nav/CMakeLists.txt`: installs `safety_gate.py`, `safety_core.py` and the safety config next to the package.

Generated (re-run the tools after editing inputs): `robot_side/config/{cad_profile.json, safety_config.yaml,
nav2_params.derived.yaml, derived_report.md}`.  `safety_config.approved` and `commissioned.json` are deliberately ABSENT:
they are the two human acts the system waits for.

To use: build the package from the copy (`colcon build --packages-select slamurai_nav`), flash the new firmware.
To go back: launch the original `slamurai_launch.py` from `Desktop/SLAMurai`, nothing else depends on this folder.
