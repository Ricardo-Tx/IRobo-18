#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import numpy
from signal import signal, SIGINT
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32

class TransformHandler():

    def __init__(self, gt_frame, est_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

        #self.warn_timer = rospy.Timer(rospy.Duration(5), self.__warn_timer_cb)

    def __warn_timer_cb(self, evt):

        available_frames = self.tf_buffer.all_frames_as_string()
        avail = True
        for frame in self.frames:
            if frame not in available_frames:
                rospy.logwarn('Frame {} has not been seen yet'.format(frame))
                avail = False
        if avail:
            self.warn_timer.shutdown()

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))


def get_errors(transform):
    tr = transform.transform.translation
    return numpy.linalg.norm( [tr.x, tr.y] )


def handle_signal(sig, frame):
    rospy.loginfo("SIGINT received, shutting down...")
    exit(0)
signal(SIGINT, handle_signal)

parser = argparse.ArgumentParser()
parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')

args, unknown = parser.parse_known_args()

gt_frame = args.gt_frame
est_frame = args.est_frame

pub = rospy.Publisher('error', Float32, queue_size=10)
rospy.init_node('evaluation_node')

if rospy.rostime.is_wallclock():
    rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
    sys.exit(1)

rospy.loginfo('Waiting for clock')
rospy.sleep(0.00001)

handler = TransformHandler(gt_frame, est_frame, max_time_between=20) # 500ms

last_eucl = 0
int_eucl = 0

msg_count = 0

rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
sleeper = rospy.Rate(10)
try:
    while not rospy.is_shutdown():
        try:
            t = handler.get_transform(gt_frame, est_frame)
        except Exception as e:
            rospy.logwarn(e)
        else:
            
            eucl = get_errors(t)
            int_eucl += (eucl+last_eucl)*0.5*0.001
            last_eucl = eucl

            pub.publish(eucl * 1e3)
            msg_count += 1
            if msg_count % 2 == 0:
                rospy.loginfo(f'Error: {eucl*1e3:.2f} mm;    Integral: {int_eucl*1e3:.2f} mm*s')

        try:
            sleeper.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)

except rospy.exceptions.ROSInterruptException:
    pass