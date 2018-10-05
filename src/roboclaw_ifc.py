#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import roboclaw_driver.roboclaw_driver as rc
from std_msgs.msg import String

MTR_ADDRESS = 128 
MAX_DUTY_CYCLE = .25 


def twist_callback(msg):

    global pub
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    TR = 3.0
    v_l = msg.linear.x + TR * msg.angular.z
    v_r = msg.linear.x - TR * msg.angular.z

    dutyleft = int( v_l * MAX_DUTY_CYCLE * 32768)
    dutyright= int( v_r * MAX_DUTY_CYCLE * 32768)
    rc.DutyM1M2(MTR_ADDRESS, dutyleft, dutyright)

    rospy.loginfo("[%d , %d]"%(dutyleft, dutyright))

    status = rc.ReadError(MTR_ADDRESS)[1]
    bvoltage =rc.ReadMainBatteryVoltage(MTR_ADDRESS)[1] / 10.0
    diagstr = "BattVoltage %f, Status 0x%x" % (bvoltage, status)

    pub.publish(diagstr)
        



def move_robot():
    global pub 
    rc.Open('/dev/ttyUSB0',115200)
    print rc.ReadMinMaxMainVoltages(MTR_ADDRESS)

    pub = rospy.Publisher('motor_diagnostics', String, queue_size=10)
  

    rospy.init_node('roboclaw_ifc')
    rospy.Subscriber("/cmd_vel", Twist, twist_callback, queue_size=1)
    rospy.spin()

    rc.DutyM1M2(MTR_ADDRESS, 0, 0)


if __name__ == '__main__':
    try:
        #Testing our function
        move_robot()
    except rospy.ROSInterruptException: pass
