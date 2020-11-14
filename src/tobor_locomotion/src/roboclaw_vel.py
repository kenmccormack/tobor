#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import roboclaw_driver.roboclaw_driver as rc
from std_msgs.msg import String

MTR_ADDRESS = 128 
MAX_DUTY_CYCLE = .75 

def timer_callback(event):
    global last_time
    if(rospy.get_rostime() - last_time ).to_sec() > 1:
       rc.DutyM1M2(MTR_ADDRESS, 0, 0)
       rospy.loginfo("timeout on control")
    
    
def move_robot():
    global pub 
    global last_time
    
    rc.Open('/dev/ttyUSB0',115200)
    print rc.ReadMinMaxMainVoltages(MTR_ADDRESS)


    rospy.init_node('roboclaw_ifc')

    vel_list = [.2, .4 , .2 , .1 , .0]

    for vel in vel_list:
        speed = int(16484 * vel)
        print speed
	rc.SpeedM1M2(MTR_ADDRESS, speed, speed)
	rospy.sleep(5)

    rc.DutyM1M2(MTR_ADDRESS, 0, 0)


if __name__ == '__main__':
    try:
        #Testing our function
        move_robot()
    except rospy.ROSInterruptException: pass
