# robot_side

Runs on the Jetson. The PC library only reaches it through the profile.

- `safety/safety_gate.py` ROS node, the only publisher of `/cmd_vel_safe`. `safety_core.py` is the pure Python logic: speed and
  acceleration limits, lidar stop distance, map fence, odometry tether, watchdogs, latched faults. `commission.py` is the human
  lidar orientation check.
- `tools/nav2_manager.py` restarts Nav2 with parameters sent from the PC. It refuses parameters beyond the approved limits
  (`limits_check.py`). `nav2_proc.py` starts and stops the Nav2 launch.
- `tools/` human steps: `make_zone.py` (safe area from the map), `approve_safety_config.py`, `arm_session.py`, `preflight.py`,
  and `derive_params.py` and `extract_cad_profile.py` (generate `config/` from the CAD).
- `config/safety_config.yaml` is generated and locked by hash. The gate starts only if a human approved that hash.
- Firmware: none needed. The gate works with the stock `control/motor_controller` odometry.

Off by default. The gate arms itself (`--auto-arm`), the zone editor (`tools/zone_editor.py`) opens when a display exists, and the cmdvel_gui sends the heartbeat while it is open, so it drives the robot through the gate. Enable with `ros2 launch slamurai_nav slamurai_launch.py safety:=true` (main.py then listens to `/cmd_vel_safe`, Nav2 is started by `nav2_manager` with the stock parameters and restarted when the PC sends new ones). Use `robotlab/profiles/slamurai.yaml`. Without `safety:=true` the robot is stock and `slamurai_stock.yaml` is the profile.

Before the first run: `python tools/preflight.py`, then commission and approve. The zone is painted live in the zone editor. Arming is automatic; `arm_session.py` stays for `--disarm` and `--clear` (a latched fault).
