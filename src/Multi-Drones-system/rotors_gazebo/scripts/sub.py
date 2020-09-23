#!/usr/bin/env python

from geometry_msgs.msg import Pose
import rospy
import math
q1=0
q2=0
q3=0
q4=0
yaw=0

def callback(msg):
    global q1,q2,q3,q4,yaw
    q1=msg.orientation.x
    q2=msg.orientation.y
    q3=msg.orientation.z
    q4=msg.orientation.w
    print(q1,q2,q3,q4)
    t4 = q1**2 + q2**2 -q3**2- q4**2
    t3 = 2*((q2*q3)+(q1*q4))
    yaw =  (math.atan2(t3, t4))
    print(yaw)


rospy.init_node("riseq_rotors_waypoint_publisher", anonymous = True)
rate = rospy.Rate(10)
odom_sub = rospy.Subscriber('/iris/ground_truth/pose', Pose, callback)
rospy.sleep(10)
# while not rospy.is_shutdown():

