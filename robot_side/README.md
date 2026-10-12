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
- Firmware is `control/motor_controller_xdrive_safe/`. It enforces its own limits and stops on silence.

Before a session: `python tools/preflight.py`, then commission, approve, make a zone, arm.
