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
        dummy,c1,c2 = rc.ReadCurrents(MTR_ADDRESS)
        bvoltage =rc.ReadMainBatteryVoltage(MTR_ADDRESS)[1] / 10.0
        diagstr = "BattVoltage %f, Current[%f,%f], Status 0x%x" % (bvoltage, c1/100.0, c2/100.0, status)
        print diagstr


if __name__ == '__main__':
    try:
        #Testing our function
        debug_robot()
    except rospy.ROSInterruptException: pass
