#!/usr/bin/env python

import rospy
import numpy as np
import os
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
from std_msgs.msg import ColorRGBA

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

def initialize_food_marker(obj_id, obj_name):
    """ 
    Initializes the obj_name (e.g. "banana") food marker.
    The marker is a red cylinder.
    """

    food_marker = Marker()
    food_marker.id = obj_id
    food_marker.header.frame_id = "/odom"    
    food_marker.ns = obj_name
    food_marker.type = Marker.CYLINDER
    food_marker.scale.x = 0.1
    food_marker.scale.y = 0.1
    food_marker.scale.z = 0.3
    food_marker.frame_locked = True
    food_marker.color.r = 1
    food_marker.color.a = 1
    return food_marker

def initialize_bot_food_marker(obj_id, obj_name):
    """ 
    Initializes the obj_name (e.g. "banana") food marker.
    The marker is a red cylinder.
    """

    bot_food_marker = Marker()
    bot_food_marker.id = obj_id
    bot_food_marker.header.frame_id = "/odom"    
    bot_food_marker.ns = "bot_" + obj_name
    bot_food_marker.type = Marker.CYLINDER
    bot_food_marker.scale.x = 0.15
    bot_food_marker.scale.y = 0.15
    bot_food_marker.scale.z = 0.3
    bot_food_marker.frame_locked = True
    bot_food_marker.color.b = 1
    bot_food_marker.color.a = 1
    return bot_food_marker

def initialize_mid_marker(obj_id, obj_name):
    """ 
    Initializes the obj_name (e.g. "banana") food marker.
    The marker is a red cylinder.
    """

    mid_marker = Marker()
    mid_marker.id = obj_id
    mid_marker.header.frame_id = "/odom"    
    mid_marker.ns = "mid_" + obj_name
    mid_marker.type = Marker.CYLINDER
    mid_marker.scale.x = 0.15
    mid_marker.scale.y = 0.15
    mid_marker.scale.z = 0.3
    mid_marker.frame_locked = True
    mid_marker.color.b = 1
    mid_marker.color.r = 1
    mid_marker.color.a = 1
    return mid_marker

def initialize_food_name_marker(food_marker):
    """ 
    Initializes the text marker associated to food_marker.
    The marker is a blue text displayed on top of food_marker.
    """

    food_name_marker = Marker()
    food_name_marker.id = food_marker.id
    food_name_marker.header.frame_id = "/odom"     
    food_name_marker.ns = food_marker.ns
    food_name_marker.type = Marker.TEXT_VIEW_FACING
    food_name_marker.scale.z = 0.2
    food_name_marker.text = food_marker.ns
    food_name_marker.pose.position.x = food_marker.pose.position.x
    food_name_marker.pose.position.y = food_marker.pose.position.y
    food_name_marker.pose.position.z = food_marker.pose.position.z
    food_name_marker.frame_locked = True
    food_name_marker.color.a = 1
    food_name_marker.color.b = 1
    return food_name_marker

class FoodViz:
    def __init__(self):
        rospy.init_node("food_viz")
        self.food = {}
        self.food_sub = {}
        self.food_viz_pub = {}
        self.bot_food_viz_pub = {}
        self.food_marker = {}
        self.bot_food_marker = {}
        self.food_name_marker = {}
        self.food_name_viz_pub = {}
        self.couple_viz_pub = {}
        self.mid_viz_pub = {}
        self.mid_marker = {}
        self.dist = {}
        self.th = {}
        self.bot_x = {}
        self.bot_y = {}
        self.bot_beta = {}
        self.couple_marker = {}
        self.object_labels = load_object_labels(PATH_TO_FOOD_LABELS)

        # Iterate over all possibly detected food objects.
        for k, v in self.object_labels.items():
            # try:
            self.food_sub[v] = rospy.Subscriber("/detector/" + v, DetectedObject, self.get_food_callback)
            self.couple_viz_pub[v] = rospy.Publisher("/viz/couple/" + v, MarkerArray, queue_size=10)
            self.mid_viz_pub[v] = rospy.Publisher("/viz/mid/" + v, Marker, queue_size=10)
            self.food_name_viz_pub[v] = rospy.Publisher("/viz/name/" + v, Marker, queue_size=10)
            self.food_marker[v] = initialize_food_marker(k, v)
            self.bot_food_marker[v] = initialize_bot_food_marker(k, v)
            self.mid_marker[v] = initialize_mid_marker(k, v)
            self.couple_marker[v] = MarkerArray() # will contain food_marker and bot_food_marker
            # except:
                # pass
        print("we are populating trans_listener")
        self.trans_listener = tf.TransformListener()

    def get_food_callback(self, food_msg):
        
        # Extract the angles associated with food_msg (raspicam message).
        # sc = food_msg.confidence 
        thetaleft = food_msg.thetaleft
        thetaright = food_msg.thetaright
        
        # Bunch of cases on thetaleft and thetaright to get the proper mid-angle, th.
        if thetaleft <= np.pi and thetaleft >= 0 and thetaright <= np.pi and thetaright >= 0:
            self.th[food_msg.name] = (thetaleft + thetaright) / 2
        if thetaleft <= 2 * np.pi and thetaleft >= np.pi and thetaright <= 2 * np.pi and thetaright >= np.pi:
            self.th[food_msg.name] = (thetaleft + thetaright) / 2       
        if thetaleft <= np.pi and thetaleft >= 0 and thetaright < 2 * np.pi and thetaright >= np.pi:
            alpha = (thetaleft / 2) + thetaright
            if alpha >= 2 * np.pi:
                self.th[food_msg.name] = alpha - 2 * np.pi
            else:
                self.th[food_msg.name] = alpha
        
        # Extract the estimated distance contained in food_msg.
        self.dist[food_msg.name] = food_msg.distance

        # Get the Turtlebot's pose in the world frame using tf.
        origin_frame = "/odom"
        (translation, rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))        
        self.bot_x[food_msg.name] = translation[0]
        self.bot_y[food_msg.name] = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.bot_beta[food_msg.name] = euler[2]

        print("robot pose in world:", (self.bot_x[food_msg.name], self.bot_y[food_msg.name], self.bot_beta[food_msg.name]))            

    def loop(self):
        try:    
            for k in self.dist:
                print("object =", k)
                # print("th =", self.th[k])

                # Assign the pose of food_marker in the world frame.
                self.food_marker[k].pose.position.x = self.bot_x[k] + self.dist[k] * np.cos(self.th[k] + self.bot_beta[k])
                self.food_marker[k].pose.position.y = self.bot_y[k] + self.dist[k] * np.sin(self.th[k] + self.bot_beta[k])               
                self.food_marker[k].pose.position.z = 0
                
                # Create the associated food_name_marker.
                self.food_name_marker[k] = initialize_food_name_marker(self.food_marker[k])

                # Create the bot marker with the last recorded position.
                self.bot_food_marker[k].pose.position.x = self.bot_x[k]
                self.bot_food_marker[k].pose.position.y = self.bot_y[k]
                self.bot_food_marker[k].pose.position.z = 0

                self.mid_marker[k].pose.position.x = self.bot_x[k] + (1./3) * self.dist[k] * np.cos(self.th[k] + self.bot_beta[k])
                self.mid_marker[k].pose.position.y = self.bot_y[k] + (1./3) * self.dist[k] * np.sin(self.th[k] + self.bot_beta[k])
                self.mid_marker[k].pose.position.z = 0
                
                self.couple_marker[k].markers = [self.food_marker[k], self.bot_food_marker[k]]

                # Publish the markers.
                self.couple_viz_pub[k].publish(self.couple_marker[k])
                self.food_name_viz_pub[k].publish(self.food_name_marker[k])
                self.mid_viz_pub[k].publish(self.mid_marker[k])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    food_viz = FoodViz()
    food_viz.run()