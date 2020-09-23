#!/usr/bin/env python


# Waypoint publisher for RotorS
# Reference: https://github.com/ethz-asl/rotors_simulator/issues/510
# 
#1-left
#3-right
import rospy
import sys
import tf
import math
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
# from rotors_msgs.msg import array_2d
# from rotors_msgs.msg import array_pub
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
#from iot_humans_track.msg import int_array1d


from sensor_msgs.msg import LaserScan
x=0
y=0
z=0
val=0
dir=0


def move(a,b,c):
	global dir
	global x
	global y
	global z
	publish_waypoint(x+a-0.12 ,y+b,2+c, 0)

def callback(msg):
	global x
	global y 
	global z
	x=msg.point.x
	y=msg.point.y
	z=msg.point.z

def publish_waypoint(x,y,z,yaw):
	"""
	Publish a waypoint to 
	"""

	command_publisher = rospy.Publisher('/iris/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)

	# create trajectory msg
	traj = MultiDOFJointTrajectory()
	traj.header.stamp = rospy.Time.now()
	traj.header.frame_id = 'frame'
	traj.joint_names.append('base_link')


	# create start point for trajectory
	transforms = Transform()
	transforms.translation.x = 0
	transforms.translation.y = 0
	transforms.translation.z = 0
	
	quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')
	transforms.rotation.x = quat[0]
	transforms.rotation.y = quat[1]
	transforms.rotation.z = quat[2]
	transforms.rotation.w = quat[3]
	
	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
	traj.points.append(point)

	# create end point for trajectory
	transforms = Transform()
	transforms.translation.x = x
	transforms.translation.y = y
	transforms.translation.z = z 

	quat = tf.transformations.quaternion_from_euler((yaw)*np.pi/180.0, 0, 0, axes = 'rzyx')
	transforms.rotation.x = quat[0]
	transforms.rotation.y = quat[1]
	transforms.rotation.z = quat[2]
	transforms.rotation.w = quat[3]

	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
	traj.points.append(point)

	rospy.sleep(1)
	command_publisher.publish(traj)

rospy.init_node("riseq_rotors_waypoint_publisher", anonymous = True)
rate = rospy.Rate(10)
# sub = rospy.Subscriber('/iris/LaserScan', LaserScan, callback1)
odom_sub = rospy.Subscriber('/iris/ground_truth/position', PointStamped, callback)
while not rospy.is_shutdown():
    move(0,0.1,0)
