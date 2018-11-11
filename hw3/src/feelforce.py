#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class ForceMapper(): 
    def __init__(self):
        self.pub_obs = rospy.Publisher('obstacle', String, queue_size=10)
        self.obstacle = ""
        self.r = rospy.Rate(250) # 250hz
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callBack)
        self.start()

    def start(self):
        while not rospy.is_shutdown():
            self.pub_obs.publish(self.obstacle)
            self.r.sleep()

    def callBack(self, msg):
        size = len(msg.ranges)
        right = msg.ranges[0]
        front90Right = msg.ranges[size/2-size/6]
        frontREighteenth = msg.ranges[size/2-size/18]
        front = msg.ranges[size/2]
        front90Left = msg.ranges[size/2+size/6]
        frontLEighteenth = msg.ranges[size/2+size/18]
        left = msg.ranges[size-1]

        if (front < 1.5):
            self.obstacle = "right"

        elif ((right > 4.5) and (left > 4.5)):
            self.obstacle = "big_right"

        elif ((frontREighteenth < 0.75)):
            self.obstacle = "right"

        elif ((frontLEighteenth < 0.75)):
            self.obstacle = "left"

        elif ((front90Left < 0.35)):
            self.obstacle = "left"

        elif ((front90Right < 0.35)):
            self.obstacle = "right"

        else:
            self.obstacle = ""

        
def main():
    rospy.init_node('ForceMapper')
    try:
        force = ForceMapper()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()