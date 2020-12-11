#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
import time
import std_msgs.msg 



class ActionQueue:

    def __init__(self):
        self.actionQueue = []
        self.status = 0
        #listen for results
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback, queue_size=1)
        self.action_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        print("created")

    def status_callback(self, msg):
        print("status {}".format(msg.status_list[0].status))
        self.status = msg.status_list[0].status
        if self.status == 3 and len(self.actionQueue) != 0:
            action = self.actionQueue.pop(0)
            
    
    def addAction(self, action):
        self.actionQueue.append(action)

    def sendAction(self, action):
        action.header = std_msgs.msg
        action.header.stamp = rospy.Time.now()
        action.header.stamp.frame_id = 'map'
        print(action)
        self.action_pub.publish(action)


    def wait(self):
        print(self.status, len(self.actionQueue))
        while self.status != 3 and len(self.actionQueue) != 0:
            time.sleep(0.1)
            


if __name__ == '__main__':
 
    #Testing our function
    print("hello")
    aq = ActionQueue()

    action = PoseStamped()
    action.pose.position.x = 1.0
    action.pose.position.y = 0.0
    action.pose.orientation.x = 0.0
    action.pose.orientation.y = 0.0
    action.pose.orientation.z = 0.0
    action.pose.orientation.w = 1.0
    aq.addAction(action)
    aq.wait()


   