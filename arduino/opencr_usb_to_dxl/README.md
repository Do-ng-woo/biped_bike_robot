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

## Built-in IMU While Running ROS

To read the OpenCR built-in IMU while controlling Dynamixels, upload this
bridge instead:

```text
../opencr_dxl_bridge_with_imu/opencr_dxl_bridge_with_imu.ino
```

It keeps normal USB-to-Dynamixel forwarding, and answers Dynamixel Protocol 2.0
read requests to virtual ID `200` with OpenCR IMU data.

Run hardware with:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  enable_opencr_imu:=true \
  opencr_imu_rate_hz:=30.0
```

Check the ROS IMU topic:

```bash
ros2 topic echo /opencr/imu
```

The bridge publishes `sensor_msgs/Imu`. Quaternion comes from the OpenCR IMU
filter, gyro is converted from deg/s to rad/s, and acceleration is converted
from g to m/s^2.

## Built-in IMU Format Check Only

The OpenCR USB serial cannot carry Dynamixel packets and debug text at the same
time. To inspect the OpenCR built-in IMU, temporarily upload:

```text
../opencr_imu_stream/opencr_imu_stream.ino
```

Then read CSV rows from the PC:

```bash
python3 src/biped_bike_robot/scripts/opencr_imu_reader.py --port /dev/opencr --rows 20
```

CSV columns:

```text
time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps,acc_x_g,acc_y_g,acc_z_g,gyro_x_adc,gyro_y_adc,gyro_z_adc,acc_x_adc,acc_y_adc,acc_z_adc
```

Re-upload `opencr_usb_to_dxl.ino` before running ROS hardware control again.
