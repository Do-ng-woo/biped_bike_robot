# OpenCR USB to Dynamixel TTL

This Arduino sketch turns an OpenCR board into a simple U2D2-style passthrough:

```text
PC /dev/ttyACM0 <-> OpenCR USB Serial <-> OpenCR Serial3 <-> Dynamixel TTL bus
```

Use it with `dxl_joint_state_bridge.py` or Dynamixel Wizard. Only one PC program
can open `/dev/ttyACM0` at a time.

## Upload

1. Open `opencr_usb_to_dxl.ino` in Arduino IDE.
2. Select `OpenCR Board`.
3. Select the OpenCR serial port.
4. Upload the sketch.
5. Close Arduino Serial Monitor and Dynamixel Wizard before running ROS.

## ROS Test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select biped_bike_robot
source install/setup.bash
ros2 launch biped_bike_robot hardware_display.launch.py max_abs_position_rad:=0.1
```

Move one slider by a small amount, such as `+0.05 rad`. If the real robot moves
in the same direction as RViz, keep `direction: 1` in `dynamixel_hardware.yaml`.
If it moves in the opposite direction, set that joint to `direction: -1`.

## Notes

- Do not print debug logs to `Serial`; it corrupts Dynamixel packets.
- The sketch powers the OpenCR Dynamixel port with `BDPIN_DXL_PWR_EN`.
- USB baudrate is mirrored to the Dynamixel TTL port. The default is `1000000`.
- Keep the robot supported while testing. A motor far from tick `2048` can move
  suddenly when torque is enabled.
