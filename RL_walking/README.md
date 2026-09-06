# RL Walking runtime

This directory is a Raspberry Pi runtime copy of the verified 50 Hz walking
pipeline. It does not import files from `mjlab` and uses the exported NumPy
policy at `models/model_90000.npz`; PyTorch is not required on the robot.

The web panel starts `run_rl_stack.py`, which owns the OpenCR serial port,
starts the policy stopped, and exposes a loopback dead-man command server on
port 8081. Never run this stack together with `hardware_display.launch.py` or
the manipulator hardware stack because only one process may own OpenCR.

Run the main package web panel from a ROS 2 Jazzy shell:

```bash
python3 scripts/web_control.py
```

Open `http://ROBOT_IP:8080`, press **Start RL Walking**, wait for READY and IMU
calibration, then hold an arrow/WASD/Q/E control. Releasing the key or losing
the browser heartbeat sends STOP. **Stop RL Walking** returns to READY before
disabling torque and releasing OpenCR.

Required Python packages are `numpy`, `PyYAML`, `dynamixel-sdk`, and the ROS 2
Jazzy Python packages used by `rclpy`.
