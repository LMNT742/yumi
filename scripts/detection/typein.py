#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    #definovanie publishera pre odosielanie sprav
    command = ""
    rospy.init_node("type_in_node")
    pub_command = rospy.Publisher("/voice_reg", String, queue_size=10)
    
    while command != "koniec":

        command = input("Zadaj naradie:")

        if command != "":
            pub_command.publish(command)
            rospy.loginfo(command)