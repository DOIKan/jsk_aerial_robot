#!/usr/bin/env python3
"""Publish one complete JointState from the two external actuator drivers."""

import threading

import rospy
from sensor_msgs.msg import JointState


class JointStateMerge:
    NAMES = ("right", "left", "gimbal1", "gimbal2", "gimbal3", "gimbal4")

    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}
        self.max_age = rospy.Duration.from_sec(float(rospy.get_param("~max_age", 0.5)))
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.joint_sub = rospy.Subscriber("robstride/joint_states", JointState, self.update, queue_size=1)
        self.gimbal_sub = rospy.Subscriber("gimbal/estimated_joint_states", JointState, self.update, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.025), self.publish)

    def update(self, state):
        if len(state.name) != len(state.position):
            return
        with self.lock:
            for name, position in zip(state.name, state.position):
                if name in self.NAMES:
                    self.values[name] = (position, state.header.stamp)

    def publish(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if any(name not in self.values or now - self.values[name][1] > self.max_age
                   for name in self.NAMES):
                return
            state = JointState()
            state.header.stamp = now
            state.name = list(self.NAMES)
            state.position = [self.values[name][0] for name in self.NAMES]
        self.pub.publish(state)


if __name__ == "__main__":
    rospy.init_node("joint_state_merge")
    JointStateMerge()
    rospy.spin()
