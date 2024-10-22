#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

counter = 0
def ekf_callback(msg, args):
    global counter
    covariance_msg = Float64MultiArray()
    pose_covariance = msg.pose.covariance
    covariance_msg.data = [
        pose_covariance[0],  # Variance of x
        pose_covariance[7],  # Variance of y
        pose_covariance[14], # Variance of z
    ]

    counter += 1

    if counter % 10 == 0:
        args.publish(Point(pose_covariance[0]*1e3, pose_covariance[7]*1e3, pose_covariance[14]*1e3))           

    rospy.loginfo(f"Published pose covariance diagonal: {covariance_msg.data}")



if __name__ == '__main__':

    rospy.init_node('cov_node')

    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(1)

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)

    pub = rospy.Publisher('cov', Point, queue_size=10)
    sub = rospy.Subscriber('odometry/filtered', Odometry, 
        callback=ekf_callback,
        callback_args=(pub)    
    )

    rospy.spin()
