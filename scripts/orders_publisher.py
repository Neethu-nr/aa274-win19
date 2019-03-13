#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker


PATH_TO_FOOD_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_food_labels.txt')

def load_object_labels(filename):
    """ loads the coco object readable name """

    fo = open(filename,'r')
    lines = fo.readlines()
    fo.close()
    object_labels = {}
    for l in lines:
        object_id = int(l.split(':')[0])
        label = l.split(':')[1][1:].replace('\n','').replace('-','_').replace(' ','_')
        object_labels[object_id] = label

    return object_labels

class OrdersPublisher:
    """
    Publishes poses of food object contained in the delivery requests 
    to the PavoneCart food delivery service. 
    """

    def __init__(self):
        rospy.init_node('orders_publisher', anonymous=True)
        self.delivery_request = None
        self.object_labels = load_object_labels(PATH_TO_FOOD_LABELS)

        self.orders_pub = rospy.Publisher('/orders', PoseArray, queue_size=10)
        self.request_sub = rospy.Subscriber('/delivery_request', String, self.get_request_callback)
        self.detected_food_sub = {}
        self.orders_points = {}
        self.request_list = None
        
        # Initialize a PoseArray msg that will contain
        # the list of requested food object poses in 
        # the world frame.
        self.orders = PoseArray()
        self.orders.header.frame_id = '/odom'

        for v in self.object_labels.values():
            self.detected_food_sub[v] = rospy.Subscriber('/viz/mid/' + v, Marker, self.get_detected_food_callback)

    def get_request_callback(self, request_msg):
        """
        Parses the request.
        e.g. 'banana,apple' -> ['banana', 'apple']
        """
        self.request_list = request_msg.split(",")

    def get_detected_food_callback(self, food_marker_msg):
        if food_marker_msg.ns in self.request_list:
            self.orders_points[food_marker_msg.ns] = Pose()
            self.orders_points[food_marker_msg.ns].position.x = food_marker_msg.pose.position.x
            self.orders_points[food_marker_msg.ns].position.y = food_marker_msg.pose.position.y
            self.orders_points[food_marker_msg.ns].position.z = food_marker_msg.pose.position.z
    
    def loop(self):
        if self.request_list:
            if len(self.orders_points) != 0:
                # Each orders_points value is a Pose msg.
                # orders_points.values is a list of Pose msgs.
                self.orders.poses = self.orders_points.values()
                self.orders_pub.publish(self.orders)

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    orders = OrdersPublisher()
    orders.run()