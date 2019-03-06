#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObject

# look up doc.
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


'''
This code identifies shiny points on the floor (aka stop_sign) and estimates its position.
It looks straight ahead within a 60 degree view, and only RHO meters in front.
Also look at velodyne_filter.launch

It broadcasts a TF frame called stop_sign, and draws a circle about the mean of the stop_sign position.

Although this "works", it is not perfect and it can be significantly improved.
- Currently, it is very noisy, as it considers other points from far away places.
- Currently, the stop_sign size is fixed.
- What happens when there are multiple nearby stop_signs?
- Does not visualize the "viewing sector" of the robot.
'''



# min number of points to be considered a stop_sign 
MIN_POINTS = 3

# look ahead distance to search for stop_signs
RHO = 0.3

# Confidence threshold above which we trigger the marking of the stop_sign
CONFIDENCE_THRESHOLD = 0.01

# min altitude of the stop sign 
MIN_ALTITUDE = 0.00

# max altitude of the stop sign 
MAX_ALTITUDE = 0.17

# opening angle of the detection volume 
ALPHA = np.pi / 6

def compute_ellipse_points(a, b):
    th = np.arange(0, 2*np.pi+1, 0.2)
    x = a * np.cos(th)
    y = b * np.sin(th)
    return np.stack([x,y])

def initialize_stop_sign_marker():
    stop_sign_marker = Marker()
    stop_sign_marker.header.frame_id = "/stop_sign"
    stop_sign_marker.ns = "ellipse"
    stop_sign_marker.type = Marker.LINE_STRIP
    stop_sign_marker.scale.x = 0.01
    stop_sign_marker.frame_locked = True
    stop_sign_marker.color.g = 1
    stop_sign_marker.color.a = 1
    return stop_sign_marker

class StopSignViz:
    def __init__(self):
        rospy.init_node("stop_sign_viz")
        self.stop_sign_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.stop_sign_mean = None
        self.stop_sign_viz_pub = rospy.Publisher("/viz/stop_sign", Marker, queue_size=10)
        self.stop_sign_marker = initialize_stop_sign_marker()
        self.confidence = None
        self.ymin = None
        self.xmin = None
        self.ymax = None
        self.xmax = None
        self.detected_object = None
        self.point_cloud = None
        rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback)
        rospy.Subscriber("/detector/stop_sign", DetectedObject , self.camera_callback)

    def camera_callback(self, msg):
        self.detected_object = msg
        if self.point_cloud != None:
            self.confidence = msg.confidence
            if self.confidence > CONFIDENCE_THRESHOLD:
                self.process_point_cloud()

    def velodyne_callback(self, msg):
        self.point_cloud = msg

    def process_point_cloud(self):
        self.ymin, self.xmin, self.ymax, self.xmax = self.detected_object.corners 
        self.point_cloud

        lidar_info = pc2.read_points(self.point_cloud, skip_nans=True, field_names=("x", "y", "z"))
        num_points = len(list(lidar_info))
        lidar_info = pc2.read_points(self.point_cloud, skip_nans=True, field_names=("x", "y", "z"))
        x_coords = np.zeros(num_points)
        y_coords = np.zeros(num_points)
        z_coords = np.zeros(num_points)
        i = 0
        # looping through the point cloud
        for p in lidar_info:
            x_coords[i] = p[0]
            y_coords[i] = p[1]
            z_coords[i] = p[2]
            i += 1

        pt_altitude = z_coords
        pt_ranges = np.hypot(x_coords, y_coords)
        pt_angles = np.arctan2(y_coords, x_coords)

        # filter based on altitude 
        pts_within_altitude = (pt_altitude > MIN_ALTITUDE) & (pt_altitude < MAX_ALTITUDE)

        # filter based on range
        pts_within_range = (pt_ranges < RHO)

        # filter based on angle
        pts_within_angle = (pt_angles < ALPHA) & (pt_angles > -ALPHA)

        # filtered points
        filtered_points = pts_within_range & pts_within_angle & pts_within_altitude

        x_filtered = x_coords[filtered_points]
        y_filtered = y_coords[filtered_points]
        z_filtered = z_coords[filtered_points]
        pts_ranges_filtered = np.hypot(x_filtered, y_filtered)
        if sum(filtered_points) > MIN_POINTS:
            max_range = np.sort(pts_ranges_filtered)[MIN_POINTS]
            pts_within_max_range = (pts_ranges_filtered < max_range)
            x_closest = x_filtered[pts_within_max_range][:MIN_POINTS]
            y_closest = y_filtered[pts_within_max_range][:MIN_POINTS]
            z_closest = z_filtered[pts_within_max_range][:MIN_POINTS]
            closest_ranges = np.hypot(x_closest, y_closest)

            self.stop_sign_time = self.point_cloud.header.stamp
            self.stop_sign_mean = (np.mean(x_closest), np.mean(y_closest), np.mean(z_closest))
            self.stop_sign_var = (np.var(x_closest), np.var(y_closest), np.var(z_closest))

    def loop(self):

        if self.stop_sign_mean is not None:
            pt = PointStamped()
            pt.header.frame_id = '/velodyne'
            pt.header.stamp = self.stop_sign_time
            pt.point.x = self.stop_sign_mean[0]
            pt.point.y = self.stop_sign_mean[1]
            pt.point.z = self.stop_sign_mean[2]

            try:
                # send a tf transform of the stop_sign location in the map frame
                self.tf_listener.waitForTransform("/map", '/velodyne', self.stop_sign_time, rospy.Duration(.05))
                stop_sign_map_pt = self.tf_listener.transformPoint("/map", pt)
                self.stop_sign_broadcaster.sendTransform((stop_sign_map_pt.point.x, stop_sign_map_pt.point.y, stop_sign_map_pt.point.z), 
                                                       [0, 0, 0, 1],
                                                       self.stop_sign_time,
                                                       "/stop_sign",
                                                       "/map")
                
                # make stop_sign marker
                ellipse_points = compute_ellipse_points(0.2, 0.2)
                self.stop_sign_marker.points = []
                for i in range(ellipse_points.shape[-1]):
                    # print("drawing ellipse")
                    self.stop_sign_marker.points.append(Point(ellipse_points[0,i], ellipse_points[1,i], 0)) 
                self.stop_sign_viz_pub.publish(self.stop_sign_marker)

            except:
                pass


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    stop_sign_viz = StopSignViz()
    stop_sign_viz.run()