import rospy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TFToPath:
    def __init__(self):
        rospy.init_node('tf_to_path_node', anonymous=True)
        
        # Create a publisher for the Path
        self.path_pub = rospy.Publisher('/ground_truth_path', Path, queue_size=10)
        
        # Initialize a Path message
        self.path = Path()
        self.path.header.frame_id = "mocap"  # Adjust the frame if necessary
        
        # Subscribe to the /tf topic
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        
        # Variable to track the last time a pose was appended
        self.last_pose_time = rospy.Time.now()

        # Spin to keep the node alive
        rospy.spin()

    def tf_callback(self, msg):
        # Process the incoming TF message
        for transform in msg.transforms:
            if transform.child_frame_id == "mocap_laser_link":  # Frame you're tracking
                # Create a PoseStamped message
                current_time = rospy.Time.now()
                
                # Only append a new pose if at least 1 second has passed since the last pose was appended
                if (current_time - self.last_pose_time).to_sec() >= 1.0:
                    pose = PoseStamped()
                    pose.header = transform.header  # Use the same timestamp and frame_id
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation

                    # Append the pose to the Path
                    self.path.poses.append(pose)
                    
                    # Limit the number of poses in the path to avoid excessive memory usage
                    MAX_PATH_LENGTH = 1000  # Adjust as necessary
                    if len(self.path.poses) > MAX_PATH_LENGTH:
                        self.path.poses.pop(0)  # Remove the oldest pose
                    
                    # Update the Path timestamp to match the transform's timestamp
                    self.path.header.stamp = transform.header.stamp  
                    
                    # Publish the Path immediately after appending the new pose
                    self.path_pub.publish(self.path)
                    
                    # Update the time the last pose was appended
                    self.last_pose_time = current_time

if __name__ == '__main__':
    try:
        TFToPath()
    except rospy.ROSInterruptException:
        pass
