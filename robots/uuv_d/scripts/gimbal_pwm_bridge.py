#!/usr/bin/env python3
"""Convert gimbal joint angles to PWM pulses for rosserial Arduino."""

import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray


class GimbalPwmBridge:
    NAMES = ("gimbal1", "gimbal2", "gimbal3", "gimbal4")

    def __init__(self):
        self.calibration = rospy.get_param("~calibration")
        if set(self.calibration) != set(self.NAMES):
            raise ValueError("calibration must contain gimbal1 through gimbal4")
        for name, cfg in self.calibration.items():
            if not (cfg["angle_min"] < cfg["angle_max"] and
                    cfg["angle_min"] < cfg["angle_center"] < cfg["angle_max"] and
                    700 <= cfg["pulse_min_us"] < cfg["pulse_center_us"] <
                    cfg["pulse_max_us"] <= 2250):
                raise ValueError("invalid gimbal calibration for " + name)
        self.lock = threading.Lock()
        self.last_applied = None
        self.ack_timeout = float(rospy.get_param("~ack_timeout", 0.5))
        self.pwm_pub = rospy.Publisher("gimbal/pwm_us", UInt16MultiArray, queue_size=1)
        self.state_pub = rospy.Publisher("gimbal/estimated_joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("gimbals_ctrl", JointState, self.command_callback, queue_size=1)
        self.applied_sub = rospy.Subscriber("gimbal/pwm_applied", UInt16MultiArray, self.applied_callback, queue_size=1)
        self.watchdog = rospy.Timer(rospy.Duration(0.1), self.check_timeout)
        rospy.logwarn("Gimbal joint states are commanded estimates, not measured angles")

    def command_callback(self, msg):
        if len(msg.name) != 4 or len(msg.position) != 4 or set(msg.name) != set(self.NAMES):
            rospy.logwarn_throttle(2.0, "gimbals_ctrl must contain all four named gimbals")
            return
        angles = dict(zip(msg.name, msg.position))
        pulses = []
        for name in self.NAMES:
            angle = angles[name]
            if not math.isfinite(angle):
                rospy.logwarn_throttle(2.0, "nonfinite gimbal angle")
                return
            cfg = self.calibration[name]
            angle = min(cfg["angle_max"], max(cfg["angle_min"], angle))
            if angle <= cfg["angle_center"]:
                ratio = (angle - cfg["angle_min"]) / (cfg["angle_center"] - cfg["angle_min"])
                start = cfg["pulse_max_us"] if cfg.get("invert", False) else cfg["pulse_min_us"]
                end = cfg["pulse_center_us"]
            else:
                ratio = (angle - cfg["angle_center"]) / (cfg["angle_max"] - cfg["angle_center"])
                start = cfg["pulse_center_us"]
                end = cfg["pulse_min_us"] if cfg.get("invert", False) else cfg["pulse_max_us"]
            pulse = round(start + ratio * (end - start))
            pulses.append(pulse)
        self.pwm_pub.publish(UInt16MultiArray(data=pulses))

    def applied_callback(self, msg):
        if len(msg.data) != 4:
            return
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.name = list(self.NAMES)
        for name, pulse in zip(self.NAMES, msg.data):
            cfg = self.calibration[name]
            if not cfg["pulse_min_us"] <= pulse <= cfg["pulse_max_us"]:
                rospy.logwarn_throttle(2.0, "Arduino returned pulse outside calibration")
                return
            center = cfg["pulse_center_us"]
            if cfg.get("invert", False):
                if pulse >= center:
                    ratio = (cfg["pulse_max_us"] - pulse) / (cfg["pulse_max_us"] - center)
                    angle = cfg["angle_min"] + ratio * (cfg["angle_center"] - cfg["angle_min"])
                else:
                    ratio = (center - pulse) / (center - cfg["pulse_min_us"])
                    angle = cfg["angle_center"] + ratio * (cfg["angle_max"] - cfg["angle_center"])
            elif pulse <= center:
                ratio = (pulse - cfg["pulse_min_us"]) / (center - cfg["pulse_min_us"])
                angle = cfg["angle_min"] + ratio * (cfg["angle_center"] - cfg["angle_min"])
            else:
                ratio = (pulse - center) / (cfg["pulse_max_us"] - center)
                angle = cfg["angle_center"] + ratio * (cfg["angle_max"] - cfg["angle_center"])
            state.position.append(angle)
        with self.lock:
            self.last_applied = time.monotonic()
        self.state_pub.publish(state)

    def check_timeout(self, _event):
        with self.lock:
            if self.last_applied is not None and time.monotonic() - self.last_applied > self.ack_timeout:
                rospy.logwarn_throttle(2.0, "Arduino PWM acknowledgement timeout")
                self.last_applied = None


if __name__ == "__main__":
    rospy.init_node("gimbal_pwm_bridge")
    try:
        GimbalPwmBridge()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Gimbal PWM bridge startup failed: %s", error)
