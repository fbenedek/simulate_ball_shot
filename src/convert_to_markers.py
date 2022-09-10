#!/usr/bin/env python3

'''
Simulates ballistic projectile shooting at the robot.
'''

PKG = 'publish_npy_stream'
ROBOT_FRAME = 'robot'
import roslib #; roslib.load_manifest(PKG)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf

MAX_LEN = 200

class ball_state_visualizer():
    def __init__(self) -> None:
        rospy.init_node('ball_array_broadcaster')
        self.sub = rospy.Subscriber("sim_capture/ball_states", Float32MultiArray, self.get_marker_array)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('sim_capture/ball_marker', Marker, queue_size=1)
        self.array_pub = rospy.Publisher('sim_capture/ball_marker_array', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()
        self.marker_idx = 0
    def run(self):
        rospy.spin()

    def get_marker_array(self, data):
        print(f"Received ball state {data.data}")
        self.tf_listener.waitForTransform("base", "odom", rospy.Time(0),rospy.Duration(4.0))
        ball_point=PointStamped()
        ball_point.header.frame_id = "base"
        ball_point.header.stamp =rospy.Time(0)
        ball_point.point.x=data.data[0]
        ball_point.point.y=data.data[1]
        ball_point.point.z=data.data[2]
        ball_point=self.tf_listener.transformPoint("odom",ball_point)        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.id = self.marker_idx
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = ball_point.point.x
        marker.pose.position.y = ball_point.point.y 
        marker.pose.position.z = ball_point.point.z
        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        self.marker_array.markers.append(marker) 
        self.pub.publish(marker)
        if len(self.marker_array.markers) > MAX_LEN:
            self.marker_array.markers.pop(0)
        self.array_pub.publish(self.marker_array)
        self.marker_idx += 1

if __name__ == '__main__':
    ball_state_viz = ball_state_visualizer()
    ball_state_viz.run()

