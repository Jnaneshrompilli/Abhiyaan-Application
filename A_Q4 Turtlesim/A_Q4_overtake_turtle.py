"""******************************************************************************************
Q4) Open turtlesim. You will be welcomed by a turtle named ‘turtle1’
    spawned at the center of the simulator.
    Spawn another turtle named 'turtle2' at (0, 5.4) at 0 degrees using the following command in the terminal
        rosservice call /spawn "x: 0.0
        y: 5.4
        theta: 0.0
        name: 'turtle2'

    Your task is to write a ROS Node using C++/Python to execute the
    following scenario:
        turtle2 starts from its initial position and travels towards turtle1.
        When turtle2 is 2 units away from turtle1, turtle2 should attempt to
        overtake turtle1. Turtle2 can move to either side and the maneuver
        could be of any shape. It should stop at a safe distance after the
        maneuver (you are free to choose the value of safe distance). Provided
        below is a reference image

Note:
    > This code works for any position of turtle1.
    > If turtle1 is in the turtle2's way turtle2 will overtake it by diverting from the its way
    

********************************************************************************************"""


#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, sin, cos


class TurtleBot:
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node("turtlebot_controller", anonymous=True)

        # Publisher which will publish to the topic '/turtle2/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            "/turtle2/cmd_vel", Twist, queue_size=10
        )

        # A subscriber to the topic '/turtle1/pose'. self.update_pose_2 is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(
            "/turtle2/pose", Pose, self.update_pose_2
        )

        # A subscriber to the topic '/turtle2/pose'. self.update_pose)1 is called when a message of type Pose is received.
        self.pose_subscriber_1 = rospy.Subscriber(
            "/turtle1/pose", Pose, self.update_pose_1
        )

        self.pose2 = Pose()
        self.pose1 = Pose()

        # Rate at which while loop executes
        self.rate = rospy.Rate(10)

    # Updating Location of turtle1
    def update_pose_1(self, data):
        self.pose1 = data
        self.pose1.x = round(self.pose1.x, 4)
        self.pose1.y = round(self.pose1.y, 4)

    # Updating location of turtle2
    def update_pose_2(self, data):
        # Callback function which is called when a new message of type Pose is
        # received by the subscriber
        self.pose2 = data
        self.pose2.x = round(self.pose2.x, 4)
        self.pose2.y = round(self.pose2.y, 4)

    # Calculating distance between goal and turtle2
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(
            pow((goal_pose.x - self.pose2.x), 2) + pow((goal_pose.y - self.pose2.y), 2)
        )

    # Calculating distance between turtle1 and turtle2
    def distance_between(self):
        return sqrt(
            pow(self.pose1.x - self.pose2.x, 2) + pow(self.pose1.y - self.pose2.y, 2)
        )

    # Calculating linear velocity of turtle2
    def linear_vel(self, goal_pose, constant=0.2):
        return constant * self.euclidean_distance(goal_pose)

    # Returns slope of the line joining turtle1 and turtle2
    def angle_difference(self):
        return atan2(self.pose1.y - self.pose2.y, self.pose1.x - self.pose2.x)

    def overtake(self):
        goal_pose = Pose()

        # x,y values as boundaries for turtle movement

        # Default values
        goal_pose.x = 9.0
        goal_pose.y = 9.0
        tolerance = 0.1

        # In case we want take input from user
        # goal_pose.x = float(input("Enter x limit (<10) : "))
        # goal_pose.y = float(input("Enter y limit (<10) : "))
        # tolerance = float(input("Set your tolerance (0.05<tol<1): "))

        # Stopping program for 2 sec
        # To get messages (location of turtles) from publishers as they publish with gap of 0.1 sec
        time.sleep(2)

        vel_msg = Twist()

        # Value of theta for turtle2 at spawing
        theta_initial = self.pose2.theta
        theta_diff_initial = self.angle_difference()

        while (
            min(abs(self.pose2.x - goal_pose.x), abs(self.pose2.y - goal_pose.y))
            >= tolerance
        ):

            # turtle2 movement untill it is 2 units away from turtle1
            if self.distance_between() >= 2:

                # Linear velocity proportional to distance between turtle2 and destination
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
            # turtle2 movement when it reaches 2 units distance from turtle1
            elif self.distance_between() < 2:

                # Angular velocity in clockwise direction to divert from the straight line
                # It will continue untill it makes PI/4 with turtle1
                if abs(self.angle_difference()) < (theta_diff_initial + 0.75):
                    vel_msg.angular.z = -0.4
                    vel_msg.linear.x = 0.5

                # After some time
                # Angular velocity in anti-clockwise to again align turtle2 in right direction
                # Condition satisfies untill it alligns in a straight line
                elif (
                    abs(self.angle_difference()) > (theta_diff_initial + 0.75)
                    and abs((self.pose2.theta - theta_initial)) > 0.01
                ):
                    vel_msg.angular.z = 0.4
                    vel_msg.linear.x = 0.5

                # Fixing angular velocity as zero to move in straight line
                else:
                    vel_msg.angular.z = 0
                    vel_msg.linear.x = self.linear_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
            else:
                pass

            self.rate.sleep()

        # Stopping our turtle2 after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        print("***** Won the Race ****")

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == "__main__":
    try:
        x = TurtleBot()
        x.overtake()
    except rospy.ROSInterruptException:
        pass
