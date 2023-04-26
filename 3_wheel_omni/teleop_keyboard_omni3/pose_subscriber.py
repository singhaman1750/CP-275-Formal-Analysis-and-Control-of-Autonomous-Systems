#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def pose_callback(data):
    # Find the index of "my_robot" in the model_states message
    index = data.name.index("my_robot")

    # Get the pose of "my_robot"
    pose = data.pose[index]

    # Print the pose
    rospy.loginfo("Robot pose: x={}, y={}, z={}".format(pose.position.x, pose.position.y, pose.position.z))

if __name__ == '__main__':
    rospy.init_node('pose_subscriber', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_callback)
    rospy.spin()
