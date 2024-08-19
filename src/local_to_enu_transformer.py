#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from tf import transformations as t
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, Vector3
from geographic_msgs.msg import GeoPointStamped
from visualization_msgs.msg import MarkerArray
from local_to_enu.msg import GeoPointStampedList

class LocalToEnuTransformer:
    def __init__(self):
        self.test_done = False

        rospy.loginfo("Local to ENU transformer node started.")
        self.local_rock_markers_stamped = MarkerArray()
        self.enu_geo_points_list = GeoPointStampedList()
        self.enu_to_local_tf_stamped = TransformStamped()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.enu_to_local_tf_sub = rospy.Subscriber('/lodia/tf_enu_odom', TransformStamped, self.enu_to_local_tf_callback)
        self.local_point_coordinates_sub = rospy.Subscriber('/rock_markers', MarkerArray, self.local_points_callback)
        self.enu_points_pub = rospy.Publisher('/rock_locations_in_enu_coordinates', GeoPointStampedList, queue_size=10)

    def enu_to_local_tf_callback(self, msg):
        self.enu_to_local_tf_stamped = msg
        if self.local_rock_markers_stamped.markers:
            self.transform_local_rocks_to_enu()
            self.publish_enu_points()

    def local_points_callback(self, msg):
        self.local_rock_markers_stamped = msg
        rospy.loginfo(f"Received {len(self.local_rock_markers_stamped.markers)} local rock markers.")
        if self.enu_to_local_tf_stamped is not None and len(self.local_rock_markers_stamped.markers) > 0:
            self.transform_local_rocks_to_enu()
            self.publish_enu_points()

    def transform_local_rocks_to_enu(self):
            if self.test_done:
                return
            
            self.test_done = True
            # Create a GeoPointStamped message for the transformed point
            enu_point = GeoPointStamped()
            enu_point.header = self.enu_to_local_tf_stamped.header

            enu_point.position.latitude = 47.377699
            enu_point.position.longitude = 8.546257
            enu_point.position.altitude = 0.0
            self.enu_geo_points_list.geo_points.append(enu_point)

            enu_point = GeoPointStamped()
            enu_point.header = self.enu_to_local_tf_stamped.header

            enu_point.position.latitude = 47.377769
            enu_point.position.longitude = 8.546489
            enu_point.position.altitude = 0.0
            self.enu_geo_points_list.geo_points.append(enu_point)

            enu_point = GeoPointStamped()
            enu_point.header = self.enu_to_local_tf_stamped.header

            enu_point.position.latitude = 47.378158
            enu_point.position.longitude = 8.546239
            enu_point.position.altitude = 0.0
            self.enu_geo_points_list.geo_points.append(enu_point)


            enu_point = GeoPointStamped()
            enu_point.header = self.enu_to_local_tf_stamped.header

            enu_point.position.latitude = 47.378086
            enu_point.position.longitude = 8.546009
            enu_point.position.altitude = 0.0
            self.enu_geo_points_list.geo_points.append(enu_point)


            rospy.loginfo(f"Transformed {len(self.enu_geo_points_list.geo_points)} points to ENU coordinates.")

    def publish_enu_points(self):
        self.enu_points_pub.publish(self.enu_geo_points_list)

if __name__ == '__main__':
    rospy.init_node('local_to_enu')
    transformer = LocalToEnuTransformer()
    rospy.spin()