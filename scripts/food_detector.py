#!/usr/bin/env python
import rospy
import numpy as np
import os
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
from std_msgs.msg import ColorRGBA

PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_food_labels.txt')

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
    food_marker = Marker()
    food_marker.id = obj_id
    food_marker.header.frame_id = "/odom"    
    food_marker.ns = obj_name
    food_marker.type = Marker.CYLINDER
    # marker.action = Marker.ADD;
    food_marker.scale.x = 0.15
    food_marker.scale.y = 0.15
    food_marker.scale.z = 0.3
    food_marker.frame_locked = True
    food_marker.color.r = 1
    # food_marker.color.g = 1
    food_marker.color.a = 1
    return food_marker

def initialize_food_name_marker(food_marker):
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
        self.food_marker = {}
        self.food_name_marker = {}
        self.food_name_viz_pub = {}
        self.dist = {}
        self.th = {}
        self.bot_x = {}
        self.bot_y = {}
        self.bot_beta = {}
        self.object_labels = load_object_labels(PATH_TO_LABELS)
        for k, v in self.object_labels.items():
            # try:
                # print("/detector/" + v)
            self.food_sub[v] = rospy.Subscriber("/detector/" + v, DetectedObject, self.get_food_callback)
            self.food_viz_pub[v] = rospy.Publisher("/viz/" + v, Marker, queue_size=10)
            self.food_name_viz_pub[v] = rospy.Publisher("/viz/name/" + v, Marker, queue_size=10)
            self.food_marker[v] = initialize_food_marker(k, v)
            # except:
                # pass
        self.trans_listener = tf.TransformListener()

    def get_food_callback(self, food_msg):
        print("in callback")
        sc = food_msg.confidence 
        thetaleft = food_msg.thetaleft
        thetaright = food_msg.thetaright
        if thetaleft <= np.pi and thetaleft >= 0 and thetaright <= np.pi and thetaright >= 0:
            print("Case 1")
            self.th[food_msg.name] = (thetaleft + thetaright) / 2
        if thetaleft <= 2 * np.pi and thetaleft >= np.pi and thetaright <= 2 * np.pi and thetaright >= np.pi:
            print("Case 2")
            self.th[food_msg.name] = (thetaleft + thetaright) / 2       
        if thetaleft <= np.pi and thetaleft >= 0 and thetaright < 2 * np.pi and thetaright >= np.pi:
            print("Case 3")
            alpha = (thetaleft / 2) + thetaright
            if alpha >= 2 * np.pi:
                print("alpha >= 0") 
                self.th[food_msg.name] = alpha - 2 * np.pi
            else:
                print("alpha < 0")
                self.th[food_msg.name] = alpha
            # self.th[food_msg.name] = (thetaleft + thetaright) / 2
 
        self.dist[food_msg.name] = food_msg.distance
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
                print("th =", self.th[k])
                self.food_marker[k].pose.position.x = self.bot_x[k] + self.dist[k] * np.cos(self.th[k] + self.bot_beta[k])
                self.food_marker[k].pose.position.y = self.bot_y[k] + self.dist[k] * np.sin(self.th[k] + self.bot_beta[k])
                # self.food_marker[k].pose.position.x = self.dist[k] * np.cos(self.th[k])
                # self.food_marker[k].pose.position.y = self.dist[k] * np.sin(self.th[k])                
                self.food_marker[k].pose.position.z = 0
                self.food_name_marker[k] = initialize_food_name_marker(self.food_marker[k])
                # send a tf transform of the puddle location in the map frame
                # self.tf_listener.waitForTransform("/map", '/raspicam', self.puddle_time, rospy.Duration(.05))
                # puddle_map_pt = self.tf_listener.transformPoint("/map", pt)
                # self.puddle_broadcaster.sendTransform((puddle_map_pt.point.x, puddle_map_pt.point.y, puddle_map_pt.point.z), 
                #                                        [0, 0, 0, 1],
                #                                        self.puddle_time,
                #                                        "/puddle",
                #                                        "/map")
                self.food_viz_pub[k].publish(self.food_marker[k])
                self.food_name_viz_pub[k].publish(self.food_name_marker[k])

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



        # try:           
        #     food_marker = self.food_marker
        #     position = self.position
        #     orientation = self.orientation
        #     food_marker.pose.position.x = position.x
        #     food_marker.pose.position.y = position.y
        #     food_marker.pose.position.z = position.z
        #     # food_marker.pose.orientation.x = orientation.x
        #     # food_marker.pose.orientation.y = orientation.y
        #     # food_marker.pose.orientation.z = orientation.z
        #     # food_marker.pose.orientation.w = orientation.w
        #     self.food_viz_pub.publish(self.food_marker)

        # except:
        #     pass
        