#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16MultiArray, String
import sys

def receive_callback(msg: String):
    received_bytes = msg.data.encode('utf-8')
    decoded_text = received_bytes.decode('utf-8')
    rospy.loginfo(decoded_text)
    if decoded_text == "koniec":
        rospy.signal_shutdown("koncim")


if __name__ == "__main__":
    rospy.init_node("receive_data")

    sub_command = rospy.Subscriber("/voice_reg", String, callback=receive_callback)

    rospy.spin()


