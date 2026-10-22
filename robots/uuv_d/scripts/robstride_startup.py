#!/usr/bin/env python3
"""Optionally enable RobStride and report the measured starting angles."""

import math
import sys

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


def main():
    rospy.init_node("robstride_startup")
    service_name = "robstride_joint_bridge/enable"
    enabled = False
    try:
        rospy.wait_for_service(service_name, timeout=15.0)
        enable = rospy.ServiceProxy(service_name, SetBool)
        response = enable(True)
        if not response.success:
            raise RuntimeError("RobStride enable failed: " + response.message)
        enabled = True

        state = rospy.wait_for_message("robstride/joint_states", JointState, timeout=5.0)
        if len(state.name) != len(state.position):
            raise RuntimeError("RobStride feedback has mismatched names and positions")
        positions = dict(zip(state.name, state.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in ("right", "left")):
            raise RuntimeError("RobStride feedback is missing a valid joint angle")
        rospy.loginfo("RobStride enabled at right=%.4f, left=%.4f rad",
                      positions["right"], positions["left"])
        return 0
    except (rospy.ROSException, rospy.ServiceException, RuntimeError, KeyError) as exc:
        rospy.logerr("RobStride startup failed: %s", exc)
        if enabled:
            try:
                enable(False)
            except rospy.ServiceException as disable_error:
                rospy.logerr("Could not disable RobStride: %s", disable_error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
