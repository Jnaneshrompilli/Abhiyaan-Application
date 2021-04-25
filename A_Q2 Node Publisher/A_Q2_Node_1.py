'''********************************************************************************
2. Write the following ROS nodes in C++/Python to perform the given
task:

 Node1
 To publish the string “Team Abhiyaan:” to the
  Topic: /team_abhiyaan
  Dtype: string

 Node2
 To publish the string “Fueled By Autonomy” to the
  Topic: /autonomy
  Dtype: string

**********************************************************************************'''


#!/usr/bin/env python

# Importing required modules
import rospy
from std_msgs.msg import String

# Function publishing topic
def talker():
    # Assigning publisher
    pub = rospy.Publisher('team_abhiyaan',String,queue_size=10)
    # Registering this node In ROS with unique identity 
    rospy.init_node('talker_n1',anonymous=True)
    # Rate at which function publishes (1 msg/sec) 
    rate = rospy.Rate(1)

    # Runs utill we shut down the Node
    while not rospy.is_shutdown():
        # Message to be published
        msg_str = "Team Abhiyaan"
        rospy.loginfo(msg_str)
        pub.publish(msg_str)
        # Rate at which loop will run
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
