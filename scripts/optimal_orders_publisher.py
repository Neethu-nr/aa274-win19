#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker
import numpy as np
from itertools import permutations

class OptimalOrdersPublisher:
    """
    Publishes the list of poses ordered optimally. 
    """
    def __init__(self):
        rospy.init_node('optimal_orders_publisher', anonymous=True)
        self.unordered_list = []
        self.ordered_list = []
        self.robot_pose = None
        self.orders_sub = rospy.Subscriber('/orders', PoseArray, self.get_orders_callback)
        self.odom_sub = rospy.Subscriber('/odom', Pose2D, self.get_pose_callback)

    def get_orders_callback(self, orders_msg):
        """
        get the orders message and order it optimally then store it in the class OptimalOrdersPublisher.
        """
        if self.unordered_list != orders_msg.poses:
            self.unordered_list = orders_msg.poses
            self.compute_optimal_order()

    def get_pose_callback(self, pose_msg):
        self.robot_pose = np.array([pose_msg.x, pose_msg.y])

    def compute_optimal_order(self):
        clean_unordered_list = [np.array([pose.position.x, pose.position.y]) for pose in self.unordered_list]
        self.ordered_list = [self.robot_pose]
        clean_unordered_list.append(np.array(self.robot_pose))
        n = len(clean_unordered_list) 
        self.distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                self.distances[i, j] = np.linalg.norm(clean_unordered_list[i] - clean_unordered_list[j], ord=1)
        possible_paths = list(set(permutations([i for i in range(n-1)])))
        best_path = min([(self.compute_distance(path), path) for path in possible_paths])[1]
        for i in best_path:
            self.ordered_list.append(clean_unordered_list[i])
        self.ordered_list.append(self.robot_pose)
        return

    def compute_distance(self, path):
        distance = 0
        distance += self.distances[n-1, path[0]]
        for i in range(len(path)-1):
            distance += self.distances[path[i], path[i+1]]
        distance += self.distances[path[-1], n-1]
        return distance

    def loop(self):
        if self.ordered_list:
            self.ordered_list_msg = PoseArray()
            self.ordered_list_msg.header.frame_id = '/odom'
            poses = []
            for pose in self.ordered_list:
                order_pose_msg = Pose()
                order_pose_msg.position.x = pose[0]
                order_pose_msg.position.y = pose[1]
                poses.append(order_pose_msg)
            self.ordered_list_msg.poses = poses

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    optimal_orders_publisher = OptimalOrdersPublisher()
    optimal_orders_publisher.run()