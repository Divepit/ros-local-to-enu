#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from tf import transformations as t
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray
from rock_detection_msgs.msg import ObstacleList
from rock_detection_msgs.srv import SetObstacleLatLon
import pymap3d as pm

class LocalToEnuTransformer:
    def __init__(self):
        rospy.init_node('local_to_enu')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.enu_to_local_tf_sub = rospy.Subscriber('/lodia/tf_enu_odom', TransformStamped, self.enu_to_local_tf_callback)
        self.local_point_coordinates_sub = rospy.Subscriber('/rock_detection/obstacle_list', ObstacleList, self.obstacles_callback) # TODO: implement checking on obstacle markers
        self.gps_reference_subscriber = rospy.Subscriber('/lodia/gps_reference_position', NavSatFix, self.gps_reference_callback)

        # self.rock_coordinates_pub = rospy.Publisher('/rock_detection/rock_coordinates', GeoPointStampedList, queue_size=10)

        self.obstacles = MarkerArray()
        # self.enu_geo_points_list = GeoPointStampedList()
        self.enu_to_local_tf_stamped = TransformStamped()
        self.inverse_enu_to_local_tf_stamped = TransformStamped()
        self.gps_reference = NavSatFix()
        # self.locked_marker_ids = []
        self.test_done = False
        
        rospy.spin()

    def enu_to_local_tf_callback(self, msg):
        self.enu_to_local_tf_stamped = msg

        self.inverse_enu_to_local_tf_stamped = self.tf_buffer.lookup_transform(self.enu_to_local_tf_stamped.child_frame_id, self.enu_to_local_tf_stamped.header.frame_id, rospy.Time(0))

    def obstacles_callback(self, msg):
        self.obstacles = msg.obstacles
        rospy.loginfo(f"Received {len(self.obstacles)} local obstacles.")
        if self.enu_to_local_tf_stamped is not None and len(self.obstacles) > 0:
            self.transform_local_rocks_to_lat_lon()

    def gps_reference_callback(self, msg):
        self.gps_reference = msg
        rospy.loginfo(f"Received GPS reference position: {self.gps_reference.latitude}, {self.gps_reference.longitude}, {self.gps_reference.altitude}")

    def transform_local_rocks_to_lat_lon(self):
        for obstacle in self.obstacles:
            if obstacle.lat == 0 or obstacle.lon == 0:
                obstacle_pose = PoseStamped()
                obstacle_pose.header.stamp = rospy.Time.now()
                obstacle_pose.header.frame_id = obstacle.header.frame_id
                obstacle_pose.pose.position.x = obstacle.x  # Changed from obstacle_pose.pose.x
                obstacle_pose.pose.position.y = obstacle.y  # Changed from obstacle_pose.pose.y
                obstacle_pose.pose.position.z = obstacle.z  # Changed from obstacle_pose.pose.z
                obstacle_coordinates_in_enu = tf2_geometry_msgs.do_transform_pose(obstacle_pose, self.inverse_enu_to_local_tf_stamped)
                lat, lon, alt = pm.enu2geodetic(
                    obstacle_coordinates_in_enu.pose.position.x,
                    obstacle_coordinates_in_enu.pose.position.y,
                    obstacle_coordinates_in_enu.pose.position.z,
                    self.gps_reference.latitude,
                    self.gps_reference.longitude,
                    self.gps_reference.altitude
                )
                self.set_obstacle_lat_lon(obstacle.obstacle_id, lat, lon)


    def publish_enu_points(self):
        self.rock_coordinates_pub.publish(self.enu_geo_points_list)

    def set_obstacle_lat_lon(self, obstacle_id, lat, lon):
        rospy.ServiceProxy('/rock_detection/set_obstacle_type', SetObstacleLatLon)
        try:
            set_obstacle_type = rospy.ServiceProxy('/rock_detection/set_obstacle_lat_lon', SetObstacleLatLon)
            set_obstacle_type(obstacle_id, lat, lon)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)

if __name__ == '__main__':
    
    transformer = LocalToEnuTransformer()
    