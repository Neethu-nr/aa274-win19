#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Pose2D
from visualization_msgs.msg import Marker
import tf
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
        self.home_pose = np.array([None, None])

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_pose_callback)
        self.orders_sub = rospy.Subscriber('/orders', PoseArray, self.get_orders_callback)
        self.optimal_orders_pub = rospy.Publisher('/orders/optimal', PoseArray, queue_size=10)

    def get_orders_callback(self, orders_msg):
        """
        get the orders message and order it optimally then store it in the class OptimalOrdersPublisher.
        """
        if self.home_pose[0]:
            if self.unordered_list != orders_msg.poses:
                self.unordered_list = orders_msg.poses
                self.compute_optimal_order()

    def get_pose_callback(self, pose_msg):
        if self.home_pose[0] == None:
            self.home_pose = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])

    def compute_optimal_order(self):
        clean_unordered_list = [np.array([pose.position.x, pose.position.y]) for pose in self.unordered_list]
        self.ordered_list = [self.home_pose]
        clean_unordered_list.append(self.home_pose)
        self.n = len(clean_unordered_list) 
        self.distances = np.zeros((self.n, self.n))
        for i in range(self.n):
            for j in range(self.n):
                self.distances[i, j] = np.linalg.norm(clean_unordered_list[i] - clean_unordered_list[j], ord=1)
        possible_paths = list(set(permutations([i for i in range(self.n-1)])))
        best_path = min([(self.compute_distance(path), path) for path in possible_paths])[1]
        for i in best_path:
            self.ordered_list.append(clean_unordered_list[i])
        self.ordered_list.append(self.home_pose)
        return

    def compute_distance(self, path):
        distance = 0
        distance += self.distances[self.n-1, path[0]]
        for i in range(len(path)-1):
            distance += self.distances[path[i], path[i+1]]
        distance += self.distances[path[-1], self.n-1]
        return distance

    def get_angle(self, pose1, pose2):
        x1, y1 = pose1.position.x, pose1.position.y
        x2, y2 = pose2.position.x, pose2.position.y
        return np.arctan2(y2 - y1, x2 - x1)

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
            
            for i in range(len(self.ordered_list_msg.poses) - 1):
                pose1 = self.ordered_list_msg.poses[i]
                pose2 = self.ordered_list_msg.poses[i + 1]
                # hack: actually treated as an angle
                self.ordered_list_msg.poses[i+1].position.z = self.get_angle(pose1, pose2)
            print("ordered_list_msg =", self.ordered_list_msg)
            self.optimal_orders_pub.publish(self.ordered_list_msg)

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    optimal_orders_publisher = OptimalOrdersPublisher()
    optimal_orders_publisher.run()