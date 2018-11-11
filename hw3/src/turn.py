#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Turn(): 
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.r = rospy.Rate(250) # 250hz
        self.move_cmd = Twist()
        self.angular_speed = 0.5
        self.turning = "False"
        self.pub_turning = rospy.Publisher('turning', String, queue_size=1)
        self.sub = rospy.Subscriber('obstacle', String, self.callBack)
        self.start()
        
    def turnCounterClockwise(self, factor):
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = abs(self.angular_speed * factor)
        self.turning = "True"

    def turnClockwise(self, factor):
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = -abs(self.angular_speed * factor)
        self.turning = "True"

    def stopTurning(self):
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.turning = "False"

    def start(self):
        while not rospy.is_shutdown():
            self.pub_turning.publish(self.turning)
            self.cmd_pub.publish(self.move_cmd)
            self.r.sleep()

    def callBack(self, msg):
        if (msg.data == "right"):
            self.turnCounterClockwise(1)

        elif (msg.data == "big_right"):
            self.angular_speed = 2
            self.turnCounterClockwise(1)
            time.sleep(.500)
            self.angular_speed = 0.5

        elif (msg.data == "left"):
            self.turnClockwise(1)
            
        else:
            self.stopTurning()
        
def main():
    rospy.init_node('Turn')
    try:
        Turn()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()