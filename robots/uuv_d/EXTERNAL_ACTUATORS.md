# 3_link external actuators

The standard `bringup.launch` still uses `servo_bridge`. On the real 3_link robot,
`external_actuators:=true` switches the joints to SocketCAN and the gimbals to
an Arduino rosserial PWM bridge. The thruster path through Spinal is unchanged.

## Prepare

1. Install `python-can` in the Python environment used by ROS
   (`python3 -c 'import rospy, can'` must succeed). The RobStride node
   loads `xiaomimotor_lib` from `/home/doi/code/robstride/xiaomimotor_lib/src`
   through `xiaomimotor_source`; change that launch argument if it moves.
2. Set up the USB-CAN adapter as `can0` at 1 Mbit/s, as described in that
   library's README. Both RobStride02 motors share this interface. `right` is
   ID 0 and `left` is ID 1.
3. The Arduino Library Manager's `Rosserial_Arduino_Library` 0.9.1 contains
   `<cstring>`, which is unavailable with this AVR toolchain. Move that
   library out of Arduino's library search directory, then generate the
   rosserial library matched to this ROS workspace:

   ```bash
   mv ~/Arduino/libraries/Rosserial_Arduino_Library ~/Arduino/Rosserial_Arduino_Library.backup
   source /home/doi/ros/jsk_aerial_robot_ws/devel/setup.bash
   rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
   ```

   Restart the Arduino IDE after generating `~/Arduino/libraries/ros_lib`.
   If the old library was not installed, skip the `mv` command.

   Flash `arduino/gimbal_pwm/gimbal_pwm.ino` to the Arduino Mega. `gimbal1`
   uses the tested pin 9; change the other three pin numbers to match wiring.
   The sketch runs rosserial at 115200 baud, which must match `arduino_baud`.
   It subscribes to `gimbal/pwm_us` (`std_msgs/UInt16MultiArray`, four pulse
   widths in microseconds) and publishes accepted values on
   `gimbal/pwm_applied`. It detaches the PWM outputs after 500 ms without a
   command or if rosserial disconnects.
4. Copy `config/3_link/RobStride.example.yaml` and
   `config/3_link/GimbalPWM.example.yaml` to calibrated files. Set each
   RobStride direction to `1` or `-1`, check motor zero offsets, and tune the
   position gains. Calibrate the actual angle to pulse mapping for every
   gimbal. The example maps -1.57/0/+1.57 rad to 700/1500/2250 microseconds,
   matching the provided single-servo test; verify the installed mechanics can safely
   travel through that range. The example RobStride file has direction `0`,
   so it will refuse to start until calibrated.

## Start

```bash
roslaunch uuv_d bringup.launch onboards_model:=3_link \
  real_machine:=true simulation:=false external_actuators:=true \
  robstride_config:=/path/to/calibrated_robstride.yaml \
  gimbal_pwm_config:=/path/to/calibrated_gimbal_pwm.yaml \
  arduino_port:=/dev/ttyACM1 arduino_baud:=115200
```

## Test actuators without Spinal or mocap

These launch files start only the named actuator driver and its ROS transport.
They do not start `aerial_robot_base_node`, Spinal, mocap, or the thrusters.

Test Arduino after flashing the sketch and calibrating the PWM mapping:

```bash
roslaunch uuv_d gimbal_arduino_test.launch \
  gimbal_pwm_config:=/path/to/calibrated_gimbal_pwm.yaml \
  arduino_port:=/dev/ttyACM1
```

In another terminal, continuously command a center pose. The Arduino detaches
the PWM outputs after 500 ms without a command:

```bash
rostopic pub -r 10 /uuv_d/gimbals_ctrl sensor_msgs/JointState \
  "{name: ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4'], position: [0.0, 0.0, 0.0, 0.0]}"
```

In a third terminal, run `rostopic echo /uuv_d/gimbal/pwm_applied`. Also check
`/uuv_d/gimbal/estimated_joint_states`. This topic reports the
angle corresponding to the acknowledged PWM, not a position sensor reading.

Test RobStride after calibrating the motor directions and offsets and bringing
up `can0`:

```bash
roslaunch uuv_d robstride_test.launch \
  robstride_config:=/path/to/calibrated_robstride.yaml
```

The motors start disabled. Add `auto_enable:=true` to make the helper wait for
the enable service, enable both motors, and report their measured starting
angles. Automatic enable is off by default. Send joint angle targets from
another terminal after checking the measured angles:

```bash
rostopic echo -n 1 /uuv_d/robstride/joint_states
rostopic pub -1 /uuv_d/joints_ctrl sensor_msgs/JointState \
  "{name: ['right', 'left'], position: [-0.02, 0.02]}"
```

The example command is suitable only if the measured starting angles are
near zero and the values are within the configured limits. Replace the
positions with safe targets close to the measured angles. The example
`command_timeout` is zero, so the bridge holds the last target until another
command or disable.

To enable manually, call
`rosservice call /uuv_d/robstride_joint_bridge/enable 'data: true'`, then inspect
`/uuv_d/robstride/joint_states` to obtain the measured starting angles. Send
`sensor_msgs/JointState` commands on `/uuv_d/joints_ctrl` using those measured
angles and small changes within the configured limits. Disable with
`rosservice call /uuv_d/robstride_joint_bridge/enable 'data: false'`.

For full hardware bringup, `robstride_auto_enable:=true` enables the motors
after the bridge starts and reports their starting angles. Leave this
argument false until the joint calibration
and the controller command path have been checked on the real mechanism.

The RobStride bridge starts with both motors disabled. After checking the
angle convention and a safe initial position, enable them with:

```bash
rosservice call /uuv_d/robstride_joint_bridge/enable 'data: true'
```

It holds the last joint target because `setJointAngle` can publish a single
message. A positive `command_timeout` enables automatic disabling after
command silence. The bridge always disables on CAN communication failure, on
a reported motor fault, and on clean shutdown. It does not re-enable motors
automatically. It uses `xiaomimotor_lib`'s control mode with configured `kp`
and `kd`; it does not change the mechanical zero.

The existing controller publishes `/uuv_d/gimbals_ctrl` (plural), which the
PWM conversion node subscribes to. A separate `rosserial_python` instance
uses the Arduino USB port. After the Arduino echoes the applied PWM values,
the conversion node publishes commanded angles as
`/uuv_d/gimbal/estimated_joint_states`; these are **not measured angles**.
The RobStride bridge publishes measured positions as
`/uuv_d/robstride/joint_states`. `joint_state_merge` publishes all six angles
together on `/uuv_d/joint_states` only while both sources are fresh.

If measured gimbal angles become available, replace the estimated state
source in `joint_state_merge`. The 3_link controller uses those angles for
thrust allocation, so a large difference between commanded and actual
gimbal angles will affect control.
