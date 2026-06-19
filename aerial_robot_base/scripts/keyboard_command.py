#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import argparse
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph



BASE_MSG = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

UUV_D_MSG = """

uuv_d mode:

     u                       i
(-roll angle)        (+pitch angle)

     j                       k
(+roll angle)        (-pitch angle)

     o
(reset roll/pitch)
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

def clamp(value, lower, upper):
        return max(lower, min(value, upper))

def applyAngleLimit(value, limit):
        if limit > 0:
                return clamp(value, -limit, limit)
        return value

def buildHelpMessage(enable_uuv_d):
        msg = BASE_MSG
        if enable_uuv_d:
                msg += UUV_D_MSG
        return msg

def parseArgs():
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--uuv-d", action="store_true", dest="uuv_d_mode",
                            help="enable roll/pitch keyboard commands for uuv_d")
        return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        args = parseArgs()
        robot_ns = rospy.get_param("~robot_ns", "");
        print(buildHelpMessage(args.uuv_d_mode))

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)
        rp_angle_step = rospy.get_param("~rp_angle_step", 0.05)
        rp_angle_limit = rospy.get_param("~rp_angle_limit", 0.0)

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        target_roll = 0.0
        target_pitch = 0.0

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        if key == 'l':
                                land_pub.publish(Empty())
                                msg = "send land command"
                        if key == 'r':
                                start_pub.publish(Empty())
                                msg = "send motor-arming command"
                        if key == 'h':
                                halt_pub.publish(Empty())
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                                msg = "send force landing command"
                        if key == 't':
                                takeoff_pub.publish(Empty())
                                msg = "send takeoff command"
                        if key == 'x':
                                motion_start_pub.publish()
                                msg = "send task-start command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                msg = "send -yaw vel command"
                                nav_pub.publish(nav_msg)
                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +z vel command"
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -z vel command"
                        if args.uuv_d_mode and key in ['u', 'j', 'i', 'k', 'o']:
                                nav_msg.roll_nav_mode = FlightNav.POS_MODE
                                nav_msg.pitch_nav_mode = FlightNav.POS_MODE
                                if key == 'u':
                                        target_roll = applyAngleLimit(target_roll - rp_angle_step, rp_angle_limit)
                                if key == 'j':
                                        target_roll = applyAngleLimit(target_roll + rp_angle_step, rp_angle_limit)
                                if key == 'i':
                                        target_pitch = applyAngleLimit(target_pitch + rp_angle_step, rp_angle_limit)
                                if key == 'k':
                                        target_pitch = applyAngleLimit(target_pitch - rp_angle_step, rp_angle_limit)
                                if key == 'o':
                                        target_roll = 0.0
                                        target_pitch = 0.0
                                nav_msg.target_roll = target_roll
                                nav_msg.target_pitch = target_pitch
                                nav_pub.publish(nav_msg)
                                msg = "send roll/pitch command (roll: {:.3f}, pitch: {:.3f})".format(target_roll, target_pitch)
                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
