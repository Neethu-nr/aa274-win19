#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import ColorRGBA

def initialize_bot_marker():
    bot_marker = Marker()
    bot_marker.header.frame_id = "/odom"
    # bot_marker.ns = "asl_turtlebot"
    bot_marker.type = Marker.CYLINDER
    # marker.action = Marker.ADD;
    bot_marker.scale.x = 0.15
    bot_marker.scale.y = 0.15
    bot_marker.scale.z = 0.3
    bot_marker.frame_locked = True
    bot_marker.color.g = 1
    bot_marker.color.a = 1
    return bot_marker

# pose: 
#   pose: 
#     position: 
#       x: 3.14991421213
#       y: 1.60000363288
#       z: -0.00100798397316
#     orientation: 
#       x: 6.06132444777e-06
#       y: 4.70813166513e-05
#       z: 0.000145631469939
#       w: 0.999999988269

class BotViz:
    def __init__(self):
        rospy.init_node("bot_viz")
        self.bot_viz_pub = rospy.Publisher("/viz/bot", Marker, queue_size=10)
        self.bot_marker = initialize_bot_marker()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        # self.trans_listener = tf.TransformListener()

    def get_odom(self, odom):
        self.position = Point()
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation

    def loop(self):
        try:           
            bot_marker = self.bot_marker
            position = self.position
            orientation = self.orientation
            bot_marker.pose.position.x = position.x
            bot_marker.pose.position.y = position.y
            bot_marker.pose.position.z = position.z
            # bot_marker.pose.orientation.x = orientation.x
            # bot_marker.pose.orientation.y = orientation.y
            # bot_marker.pose.orientation.z = orientation.z
            # bot_marker.pose.orientation.w = orientation.w
            self.bot_viz_pub.publish(self.bot_marker)

        except:
            pass
        
        # try:
        #     bot_marker = self.bot_marker
        #     origin_frame = "/odom"
        #     (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
        #     bot_marker.pose.position.x = translation[0]
        #     bot_marker.pose.position.y = translation[1]
        #     # euler = tf.transformations.euler_from_quaternion(rotation)
        #     # self.theta = euler[2]
        #     self.bot_viz_pub.publish(self.bot_marker)

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    bot_viz = BotViz()
    bot_viz.run()