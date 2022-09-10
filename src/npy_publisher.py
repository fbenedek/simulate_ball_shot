#!/usr/bin/env python3
PKG = 'publish_npy_stream'
import roslib #; roslib.load_manifest(PKG)

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def talker():
    pub = rospy.Publisher('sim_capture/ball_states', Float32MultiArray, queue_size=10)
    rospy.init_node('npy_publisher', anonymous=True)
    r = rospy.Rate(50) # 10hz
    array = np.load('ball_inputs.npy')
    i = 0
    while not rospy.is_shutdown():
        if i < array.shape[0]:
            msg = Float32MultiArray()
            msg.data = array[i,:]
            pub.publish(msg)
            i += 1
            r.sleep()

if __name__ == '__main__':
    talker()