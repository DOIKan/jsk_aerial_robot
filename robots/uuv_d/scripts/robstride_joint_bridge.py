#!/usr/bin/env python3
"""Drive the two uuv_d link joints through xiaomimotor_lib on SocketCAN."""

import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse


class RobStrideJointBridge:
    def __init__(self):
        try:
            from xiaomimotor import CanMotorController
        except ImportError as exc:
            raise RuntimeError(
                "Could not import xiaomimotor or one of its dependencies: {}".format(exc)
            ) from exc

        self.lock = threading.RLock()
        self.bus = rospy.get_param("~can_interface", "can0")
        self.rate = float(rospy.get_param("~rate", 25.0))
        # Joint commands are often sent once by RobotInterface.setJointAngle.
        # Zero keeps the last target until disabled; a positive value opts in
        # to disabling after command silence.
        self.timeout = float(rospy.get_param("~command_timeout", 0.0))
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout", 0.25))
        if self.rate <= 0 or self.timeout < 0 or self.feedback_timeout <= 0:
            raise ValueError("invalid RobStride rate or timeout")
        configs = rospy.get_param("~motors")
        if set(configs) != {"right", "left"}:
            raise ValueError("~motors must configure right and left")
        ids = [int(configs[name]["id"]) for name in ("right", "left")]
        if len(set(ids)) != 2 or any(i < 0 or i > 127 for i in ids):
            raise ValueError("right and left need different CAN IDs in 0..127")

        self.motors = {}
        self.id_to_name = {}
        self.limits = {}
        self.targets = {}
        self.feedback = {}
        self.last_command = None
        self.enabled = False
        for name in ("right", "left"):
            cfg = configs[name]
            direction = int(cfg["direction"])
            if direction not in (-1, 1):
                raise ValueError("motor direction must be -1 or 1")
            lower, upper = float(cfg["lower"]), float(cfg["upper"])
            if lower >= upper:
                raise ValueError("invalid angle range for " + name)
            motor = CanMotorController(
                bus=self.bus, motor_id=int(cfg["id"]),
                motor_type=cfg["type"], motor_dir=direction,
            )
            motor.set_angle_offset(float(cfg["offset"]))
            motor.set_angle_range(lower, upper)
            self.motors[name] = motor
            self.id_to_name[int(cfg["id"])] = name
            self.limits[name] = (lower, upper)
            self.targets[name] = None
        self.kp = {name: float(configs[name]["kp"]) for name in self.motors}
        self.kd = {name: float(configs[name]["kd"]) for name in self.motors}

        self.state_pub = rospy.Publisher("robstride/joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("joints_ctrl", JointState, self.command_callback, queue_size=1)
        self.enable_srv = rospy.Service("~enable", SetBool, self.enable_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.update)
        rospy.on_shutdown(self.stop_motors)
        rospy.loginfo("RobStride joint bridge ready on %s; motors are disabled", self.bus)

    def command_callback(self, msg):
        if len(msg.name) != len(msg.position):
            rospy.logwarn_throttle(2.0, "joints_ctrl name/position lengths differ")
            return
        with self.lock:
            accepted = False
            for name, angle in zip(msg.name, msg.position):
                if name not in self.motors or not math.isfinite(angle):
                    rospy.logwarn_throttle(2.0, "invalid RobStride joint command")
                    continue
                lower, upper = self.limits[name]
                self.targets[name] = min(upper, max(lower, angle))
                accepted = True
            if accepted:
                self.last_command = time.monotonic()

    def enable_callback(self, request):
        with self.lock:
            if not request.data:
                self.stop_motors()
                return SetBoolResponse(success=True, message="RobStride motors disabled")
            if self.enabled:
                return SetBoolResponse(success=True, message="already enabled")
            try:
                for name, motor in self.motors.items():
                    _, error, position, velocity, effort, _ = motor.enable_motor()
                    if position is None or error:
                        raise RuntimeError("no valid enable feedback from " + name)
                    if not self.limits[name][0] <= position <= self.limits[name][1]:
                        raise RuntimeError("measured {} angle is outside the configured limits".format(name))
                    mode_feedback = motor.set_run_mode("CONTROL_MODE")
                    if mode_feedback is None or mode_feedback[0] is None:
                        raise RuntimeError("could not set CONTROL_MODE on " + name)
                    self.targets[name] = position
                    self.feedback[name] = (position, velocity, effort, time.monotonic())
                self.enabled = True
                self.last_command = time.monotonic()
                return SetBoolResponse(success=True, message="RobStride motors enabled")
            except Exception as exc:
                self.stop_motors()
                return SetBoolResponse(success=False, message=str(exc))

    def stop_motors(self):
        with self.lock:
            self.enabled = False
            for motor in self.motors.values():
                try:
                    motor.disable_motor()
                except Exception as exc:
                    rospy.logerr("Could not disable RobStride motor: %s", exc)

    def update(self, _event):
        with self.lock:
            now = time.monotonic()
            if not self.enabled:
                return
            if self.timeout > 0 and (self.last_command is None or now - self.last_command > self.timeout):
                rospy.logerr("RobStride command timeout; disabling motors")
                self.stop_motors()
                return
            try:
                for name, motor in self.motors.items():
                    if not motor.only_send_control_command(
                        p_ref=self.targets[name], v_ref=0.0,
                        kp=self.kp[name], kd=self.kd[name], tau_ff=0.0,
                    ):
                        raise RuntimeError("CAN send failed for " + name)
                pending = set(self.motors)
                deadline = time.monotonic() + self.feedback_timeout
                while pending and time.monotonic() < deadline:
                    motor_id, data, arbitration_id = self.motors["right"].receive_can_raw_message(
                        self.bus, timeout=5)
                    if (motor_id not in self.id_to_name or data is None or len(data) != 8 or
                            (arbitration_id >> 24) != 2):
                        continue
                    name = self.id_to_name[motor_id]
                    _, error, position, velocity, effort, _ = self.motors[name].parse_received_msg(
                        data, arbitration_id)
                    if error:
                        raise RuntimeError("RobStride fault on {}: {}".format(name, error))
                    self.feedback[name] = (position, velocity, effort, time.monotonic())
                    pending.discard(name)
                if pending:
                    raise RuntimeError("CAN feedback timeout: " + ", ".join(sorted(pending)))
                state = JointState()
                state.header.stamp = rospy.Time.now()
                for name in ("right", "left"):
                    position, velocity, effort, _ = self.feedback[name]
                    state.name.append(name)
                    state.position.append(position)
                    state.velocity.append(velocity)
                    state.effort.append(effort)
                self.state_pub.publish(state)
            except Exception as exc:
                rospy.logerr("RobStride bridge fault: %s", exc)
                self.stop_motors()


if __name__ == "__main__":
    rospy.init_node("robstride_joint_bridge")
    try:
        RobStrideJointBridge()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("RobStride bridge startup failed: %s", error)
