#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import ColorRGBA, Header
import matplotlib.pyplot as plt

def initialize_stop_sign_marker():
    robot_marker = Marker()
    robot_marker.header.frame_id = "/map"
    robot_marker.type = Marker.SPHERE
    robot_marker.action = Marker.ADD
    robot_marker.scale.x = 0.2
    robot_marker.scale.y = 0.2
    robot_marker.scale.z = 0.2
    robot_marker.color.r = 1.0
    robot_marker.color.a = 1.0
    robot_marker.text = "marker"
    return robot_marker

# def initialize_cmd_pose_marker():
#     robot_marker = Marker()
#     robot_marker.header.frame_id = "/map"
#     robot_marker.type = Marker.SPHERE
#     robot_marker.action = Marker.ADD
#     robot_marker.scale.x = 0.1
#     robot_marker.scale.y = 0.1
#     robot_marker.scale.z = 0.1
#     robot_marker.color.g = 1.0
#     robot_marker.color.a = 1.0
#     robot_marker.text = "marker"
#     return robot_marker

class StopSignMarker():
    def __init__(self):
        rospy.init_node("stop_sign_marker")
        self.stop_sign_marker_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.stop_sign_marker_mean = None
        self.x = None
        self.y = None
        self.z = None
        self.theta = None
        self.orientation_x = None
        self.orientation_y = None
        self.orientation_z = None
        self.orientation_w = None
        self.robot_marker_viz_pub = rospy.Publisher("/viz/robot_marker", Marker, queue_size=10)
        self.cmd_pose_viz_pub = rospy.Publisher("/viz/cmd_pose", Marker, queue_size=10)
        self.robot_marker = initialize_robot_marker()
        self.cmd_pose_marker = initialize_cmd_pose_marker()
        rospy.Subscriber('/odom', Odometry, self.robot_marker_callback)
        rospy.Subscriber('/cmd_pose', Pose2D, self.cmd_pose_marker_callback)

    def robot_marker_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        self.orientation_x = data.pose.pose.orientation.x
        self.orientation_y = data.pose.pose.orientation.y
        self.orientation_z = data.pose.pose.orientation.z
        self.orientation_w = data.pose.pose.orientation.w

    def cmd_pose_marker_callback(self, data):
        self.x_cmd_pose = data.x
        self.y_cmd_pose = data.y
        

    # def robot_marker_callback(self, data):
    #     if "turtlebot3_burger" in data.name:
    #         pose = data.pose[data.name.index("turtlebot3_burger")]
    #         twist = data.twist[data.name.index("turtlebot3_burger")]
    #         self.x = pose.position.x
    #         self.y = pose.position.y
    #         quaternion = (
    #             pose.orientation.x,
    #             pose.orientation.y,
    #             pose.orientation.z,
    #             pose.orientation.w)
    #         euler = tf.transformations.euler_from_quaternion(quaternion)
    #         self.theta = euler[2]
    #         self.orientation_x = pose.orientation.x
    #         self.orientation_y = pose.orientation.y
    #         self.orientation_z = pose.orientation.z
    #         self.orientation_w = pose.orientation.w


    def loop(self):
        if (self.x is not None) and (self.y is not None) and (self.theta is not None):
            try:
                self.robot_marker.pose.orientation.x = self.orientation_x
                self.robot_marker.pose.orientation.y = self.orientation_y
                self.robot_marker.pose.orientation.z = self.orientation_z
                self.robot_marker.pose.orientation.w = self.orientation_w
                self.robot_marker.pose.position.x = self.x
                self.robot_marker.pose.position.y = self.y
                self.robot_marker.pose.position.z = 0.0
                self.robot_marker_viz_pub.publish(self.robot_marker)


                self.cmd_pose_marker.pose.position.x = self.x_cmd_pose
                self.cmd_pose_marker.pose.position.y = self.y_cmd_pose
                self.cmd_pose_marker.pose.position.z = 0.0
                self.cmd_pose_marker_viz_pub.publish(self.cmd_pose_marker)
            except:
                pass

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    robot_marker = RobotMarker()
    robot_marker.run()

