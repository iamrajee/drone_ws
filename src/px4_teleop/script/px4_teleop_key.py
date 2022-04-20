#!/usr/bin/env python

import math

import curses
import rospy

from geometry_msgs.msg import PoseStamped

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest

from mavros_msgs.msg import State

current_state = State()

def addstrln(scr, string, posx=0, posy=0):
    scr.addstr(posx, posy, string)
    scr.insertln()


def show_key_config(scr):
    addstrln(scr, "ctrl-c: quit")
    addstrln(scr, "right arrow: turn right")
    addstrln(scr, "left arrow: turn left")
    addstrln(scr, "up arrow: upward")
    addstrln(scr, "down arrow: downward")
    addstrln(scr, "d: right")
    addstrln(scr, "s: backward")
    addstrln(scr, "a: left")
    scr.addstr(0, 0, "w: forward")


def publish_pos(pub, msg, key):
    if key==119:
        # w
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x += 0.1
        pub.publish(msg)
    elif key==97:
        # a
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.y += 0.1
        pub.publish(msg)
    elif key==115:
        # s
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x -= 0.1
        pub.publish(msg)
    elif key==100:
        # d
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.y -= 0.1
        pub.publish(msg)
    elif key==260:
        # <-
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.z = math.sin(0.5)
        msg.pose.orientation.w = math.cos(0.5)
        pub.publish(msg)
    elif key==259:
        # up arrow
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.z += 0.1
        pub.publish(msg)
    elif key==261:
        # ->
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.z = math.sin(-0.5)
        msg.pose.orientation.w = math.cos(-0.5)
        pub.publish(msg)
    elif key==258:
        # down arrow
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.z -= 0.1
        pub.publish(msg)
    else:
        pub.publish(msg)


def state_cb(msg):
    current_state = msg


def px4_teleop_key():
    rospy.init_node("px4_teleop_key_pub", anonymous=True)
    rospy.loginfo("Node Initialized")

    state_sub = rospy.Subscriber("mavros/state", State, state_cb, queue_size=10)
    pos_teleop_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.loginfo("Subscriber, Publisher and Service Clients are initilized")

    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and current_state.connected:
        rate.sleep()

    pos_teleop_msg = PoseStamped()
    pos_teleop_msg.header.stamp = rospy.Time.now()
    pos_teleop_msg.pose.position.x = 0.
    pos_teleop_msg.pose.position.y = 0.
    pos_teleop_msg.pose.position.z = 2.0 
    
    for i in range(100):
        pos_teleop_pub.publish(pos_teleop_msg)
        rate.sleep()

    set_mode_req = SetModeRequest()
    set_mode_req.custom_mode = "OFFBOARD"

    arm_cmd_req = CommandBoolRequest()
    arm_cmd_req.value = True

    while not set_mode_client(set_mode_req).success:
        pass

    rospy.loginfo("Offboard enabled")
        
    while not arming_client(arm_cmd_req).success:
        pass

    rospy.loginfo("Vehicle armed")

    pos_teleop_pub.publish(pos_teleop_msg)

    try:
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(1)
        show_key_config(stdscr)

        while not rospy.is_shutdown():
            if not current_state.mode=="OFFBOARD":
               set_mode_client(set_mode_req)
            op = stdscr.getch()
            publish_pos(pos_teleop_pub, pos_teleop_msg, op)
            rate.sleep()

    finally:
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()

if __name__=="__main__":
    try:
        px4_teleop_key()
    except rospy.ROSInterruptException:
        pass
