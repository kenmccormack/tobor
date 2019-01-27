#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Listener heard: %s", data.data)
    
def listener():

   
    rospy.Subscriber("chatter", String, callback)

    rospy.spin()

def talker():
    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "sending message: hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__': 
    rospy.init_node('remote_chatter', anonymous=True)

    mode = rospy.get_param("~mode", "listener")

    if mode=="listener":
        listener()
    else:
        talker()

   



  