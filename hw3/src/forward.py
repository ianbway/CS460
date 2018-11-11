#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Forward(): 
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.r = rospy.Rate(250) # 250hz
        self.linear_speed = 0.25
        self.move_cmd = Twist()
        self.sub = rospy.Subscriber('turning', String, self.callBack)
        self.start()
       
    def forward(self):
        self.move_cmd.linear.x = self.linear_speed

    def stop(self):
        self.move_cmd.linear.x = 0

    def start(self):
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.move_cmd)
            self.r.sleep()

    def callBack(self, msg):
        if (msg.data == "True"):
            self.stop()
        else:
            self.forward()
        
def main():
    rospy.init_node('Forward')
    try:
        Forward()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()