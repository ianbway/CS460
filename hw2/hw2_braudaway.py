#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty as EmptyServiceCall

# Ian Braudaway

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)
        self.vel_msg = Twist()

        self.turtle2_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        
        self.clear_background = rospy.ServiceProxy('clear', EmptyServiceCall)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        # Add code to update self.pose

        self.pose = data

        # The following function will be used to add a small amount of noise 
        self.applyMomentum(self.pose.x, 5, self.pose.theta, self.pose.linear_velocity) 
    
    def applyMomentum(self, x, y, z, mag):
        # adds noise to robot position
        robot.turtle2_teleport(x + mag, y, z)

    def stop(self):
        # Stop robot after the movement is over.
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)

    def setLinearVelocity(self, u):
        # Set turtle linear velocity to X
        self.vel_msg.linear.x = u
        self.velocity_publisher.publish(self.vel_msg)

    def getGoalPose(self, x, y):
        # Create and return goal_pose object (hint: Pose())
        goalPose = Pose()
        goalPose.x = x
        goalPose.y = y
        return goalPose

    def getTime(self, start):
        now = rospy.get_rostime()
        curTime  = now.secs - start.secs
        return curTime

    def getError(self, goalX, robotX):
        # Return error between the goal X position and the turtle x position
        error = float(goalX) - float(robotX)
        return error

    def move2goal(self, x, y):
        goal_pose = self.getGoalPose(x, y)
        start = rospy.get_rostime()
        linearVelocity = 1

        while abs(linearVelocity) > 0:
            error = self.getError(goal_pose.x, self.pose.x)

            # Use error to generate proportional control signal that modifies the turtle X         
            # position. Hint: use setLinearVelocity function
            # Your solution must include a Kp gain term

            rospy.loginfo("X: %f, Y: %f, Time: %i", self.pose.x, self.pose.y, self.getTime(start))

            # Update linearVelocity to equal your control signal
            K_P = 0.35
            linearVelocity = float(K_P) * float(error)

            # Note: you also need to call self.setLinearVelocity() within this loop
            
            self.setLinearVelocity(linearVelocity)

            self.clear_background()
            self.rate.sleep()
        
        self.stop()
        now = rospy.get_rostime()
        compTime = now.secs - start.secs
        rospy.loginfo("Total time: %i", compTime)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = TurtleBot()
        robot.turtle2_teleport(0,5,0);
        robot.move2goal(5,5)
    except rospy.ROSInterruptException:
        pass
