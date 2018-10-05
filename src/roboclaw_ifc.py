#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import roboclaw_driver.roboclaw_driver as rc

MTR_ADDRESS = 128 
MAX_DUTY_CYCLE = .5 

def twist_callback(msg):


    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    TR = 1.0
    v_l = msg.linear.x - TR * msg.angular.z
    v_r = msg.linear.x + TR * msg.angular.z

    dutyleft = int( v_l * MAX_DUTY_CYLE * 32768)
    dutyright= int( v_r * MAX_DUTY_CYLE * 32768)
    roboclaw.DutyM1M2(MTR_ADDRESS, dutyleft, dutyright)

    rospy.loginfo([%d , %d]%(dutyleft, dutyright))



def move_robot():
    rc.Open('/dev/ttyUSB0',115200)
    print rc.ReadMinMaxMainVoltages(MTR_ADDRESS)

    rospy.init_node('roboclaw_ifc')
    rospy.Subscriber("/cmd_vel", Twist, twist_callback)
    rospy.spin()



if __name__ == '__main__':
    try:
        #Testing our function
        move_robot()
    except rospy.ROSInterruptException: pass
