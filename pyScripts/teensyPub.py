#!/usr/bin/env python
import rospy
from ralp_msgs.msg import teensy_input

def talker():
    pub = rospy.Publisher('ralp_msgs', teensy_input)
    rospy.init_node('ralp_msgs', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = teensy_input()
    msg.buttons = 1 
    msg.deltax = 0.001 
    msg.deltay = 0.001

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass