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


def tf_callback(msg, args):
    detect_parent = args[0]
    detect_child = args[1]
    pub_parent = args[2]
    pub_child = args[3]
    handler = args[4]
    publisher = args[5]
    for transform in msg.transforms:
        if transform.header.frame_id == detect_parent and transform.child_frame_id == detect_child:
            # rospy.loginfo("Matching transform found!")
            try:
                tr = handler.get_transform(pub_parent, pub_child).transform.translation
                publisher.publish(Point(tr.x,tr.y,tr.z))
            except:
                pass
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--base_frame', help='The child frame of the GT transform', default='mocap')
    parser.add_argument('--gt_frame', help='The child frame of the GT transform', default='mocap_laser_link')
    parser.add_argument('--est_frame', help='The child frame of the estimation transform', default='base_scan')

    args, unknown = parser.parse_known_args()


    rospy.init_node('point_node')


    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(1)

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)


    gt_handler = TransformHandler(args.base_frame, args.gt_frame, max_time_between=20) # 500ms
    est_handler = TransformHandler(args.base_frame, args.est_frame, max_time_between=20) # 500ms

    gt_pub = rospy.Publisher('gt_points', Point, queue_size=10)
    est_pub = rospy.Publisher('est_points', Point, queue_size=10)

    gt_sub = rospy.Subscriber("/tf", TFMessage, 
        callback=tf_callback, 
        callback_args=('mocap', 'mocap_laser_link', 'mocap', 'mocap_laser_link', gt_handler, gt_pub)
    )
    est_sub = rospy.Subscriber("/tf", TFMessage, 
        callback=tf_callback, 
        callback_args=('odom', 'base_footprint', 'mocap', 'base_link', est_handler, est_pub)
    )

    # gt_transform = gt_handler.get_transform(args.base_frame, args.gt_frame)
    # est_transform = est_handler.get_transform(args.base_frame, args.est_frame)


    rospy.spin()
