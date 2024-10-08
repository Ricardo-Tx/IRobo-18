#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class EKFWithPath:
    def __init__(self):
        rospy.init_node('ekf_with_path_node', anonymous=True)

        # Create a publisher for the Path
        self.path_pub = rospy.Publisher('/filtered_path', Path, queue_size=10)

        # Initialize a Path message
        self.path = Path()
        self.path.header.frame_id = "filtered_path"  # Adjust if necessary

        # List to keep track of positions
        self.position_history = []
        
        # Subscribe to the /odometry/filtered topic
        rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)

        # Timer to publish the path every second
        rospy.Timer(rospy.Duration(1.0), self.publish_path)

    def odometry_callback(self, msg):
        # Extract the position from the Odometry message
        position = msg.pose.pose.position  # Getting position from the Odometry message

        # Store the estimated position
        self.position_history.append((position.x, position.y, position.z))  # Store as a tuple

        # Limit the number of positions to store (optional)
        if len(self.position_history) > 1000:  # Adjust as necessary
            self.position_history.pop(0)  # Remove the oldest position

    def publish_path(self, event):
        # Create a PoseStamped message for the Path
        if self.position_history:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()  # Set the current timestamp
            pose.header.frame_id = self.path.header.frame_id

            # Use the latest estimated position
            latest_position = self.position_history[-1]
            pose.pose.position.x = latest_position[0]
            pose.pose.position.y = latest_position[1]
            pose.pose.position.z = latest_position[2]
            pose.pose.orientation.w = 1.0  # Assuming no orientation information is available

            # Append the pose to the Path
            self.path.poses.append(pose)

            # Publish the Path
            self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        ekf_with_path = EKFWithPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
