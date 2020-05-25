#!/usr/bin/env python

'''
This is being written in python since we are trying to implement openCAN in python.
This can be easily rewritten in C++
'''

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('jetson_comms/can', String, queue_size=10)
    rospy.init_node('can_raw', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "CAN DATA %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
