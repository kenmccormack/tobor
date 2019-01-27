#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import roboclaw_driver.roboclaw_driver as rc
from std_msgs.msg import String

MTR_ADDRESS = 128 
MAX_DUTY_CYCLE = .55 





def debug_robot():

    rc.Open('/dev/ttyUSB0',115200)
  
    while True:
        status = rc.ReadError(MTR_ADDRESS)[1]
	encoder = rc.ReadEncM1(MTR_ADDRESS)
	print encoder


if __name__ == '__main__':
    try:
        #Testing our function
        debug_robot()
    except rospy.ROSInterruptException: pass
