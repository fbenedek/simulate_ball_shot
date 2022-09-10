#!/usr/bin/env python3

'''
Simulates ballistic projectile shooting at the robot.
'''

PKG = 'publish_npy_stream'
ROBOT_FRAME = 'robot'
import roslib #; roslib.load_manifest(PKG)
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Vector3Stamped

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf

class ball_state_visualizer():
    def __init__(self):
        rospy.init_node('ball_simulator')
        self.sub = rospy.Subscriber("sim_capture/fire_ball_at_robot", Float32MultiArray, self.fire_ball)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('sim_capture/ball_states', Float32MultiArray, queue_size=1)
        self.ball_position = np.array([0.0,0.0,0.0])
        self.ball_speed = np.array([0.0,0.0,0.0])
        self.intersection_pt = np.array([0,0,0.98])
        self.ball_acceleration = np.array([0,0,-9.81])
        self.initial_ball_pos = np.array([4.0,0,0.0])
        self.rate = 100
        self.reset_stepts = 400
        self.steps_since_last_shot = 0
        self.ball_height = 1.1

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.ball_position += 1/self.rate * self.ball_speed
            if self.ball_position[2] < 0:
                self.ball_speed = np.array([0.0,0.0,0.0])
            else:
                self.ball_speed += 1/self.rate * self.ball_acceleration
            msg = Float32MultiArray()
            if self.steps_since_last_shot > self.reset_stepts:
                self.intersection_pt[1] = 0.0
            self.steps_since_last_shot += 1
            try:
                self.tf_listener.waitForTransform("odom", "base", rospy.Time(0),rospy.Duration(4.0))
                ball_pos_base, ball_speed_base = self.convert_ball_state()
                ball_pos_list = ball_pos_base.tolist()
                ball_speed_list = ball_speed_base.tolist()
                intersection_pt_list = self.intersection_pt.tolist()
                msg.data = (ball_pos_list[0],ball_pos_list[1],ball_pos_list[2],
                    ball_speed_list[0], ball_speed_list[1], ball_speed_list[2],
                    intersection_pt_list[0], intersection_pt_list[1], intersection_pt_list[2])
                self.pub.publish(msg)
            except:
                pass
                    
            rate.sleep()

    def convert_ball_state(self):
        self.tf_listener.waitForTransform("odom", "base", rospy.Time(0),rospy.Duration(4.0))
        ball_point=PointStamped()
        ball_point.header.frame_id = "odom"
        ball_point.header.stamp =rospy.Time(0)
        ball_point.point.x=self.ball_position[0]
        ball_point.point.y=self.ball_position[1]
        ball_point.point.z=self.ball_position[2]
        p=self.tf_listener.transformPoint("base",ball_point)
        ball_speed=Vector3Stamped()
        ball_speed.header.frame_id = "odom"
        ball_speed.header.stamp =rospy.Time(0)
        ball_speed.vector.x=self.ball_speed[0]
        ball_speed.vector.y=self.ball_speed[1]
        ball_speed.vector.z=self.ball_speed[2]
        v=self.tf_listener.transformVector3("base",ball_speed)
        return np.array([p.point.x, p.point.y, p.point.z]), np.array([v.vector.x, v.vector.y, v.vector.z])

    def fire_ball(self, data):
        try:
            self.steps_since_last_shot = 0
            (trans,rot) = self.tf_listener.lookupTransform('base', 'odom', rospy.Time(0))
            print(trans)
            print(rot)
            displacement, speed = data.data
            print("Firing ball to displacement " + str(displacement) + " with speed " + str(speed))
            # set speed, initial position
            self.ball_speed[0] = - speed
            self.ball_speed[2] = self.initial_ball_pos[0]/speed * ( - self.ball_acceleration[2] * 0.5)
            self.ball_position = self.initial_ball_pos.copy()
            self.ball_position[1] += displacement
            print("height and trans is " + str(self.ball_height) + " trans " + str(trans[2]))
            self.intersection_pt[1] = displacement
            # get the ball in the world CS
            self.ball_position += np.array(trans)
            self.ball_position[2] = self.ball_height
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    ball_state_viz = ball_state_visualizer()
    ball_state_viz.run()


# rostopic pub /sim_capture/fire_ball_at_robot std_msgs/Float32MultiArray "data: [0.3, 10.1]"