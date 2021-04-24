'''*********************************************************************************
3) Write a ROS node in C++/Python that listens to both the topics
 published in the previous question:
 Topic:
 /team_abhiyaan
 Dtype:
 string

 Topic:
 /autonomy
 Dtype:
 string
  and appends the content of Node 1 and Node 2 and prints on the
  terminal. The final message should be like “Team Abhiyaan: Fueled By
  Autonomy”
*************************************************************************************'''


#!/usr/bin/env python

# Importing required modules
import rospy
import message_filters
from std_msgs.msg import String


# Function to print the message
def callback(team_abhiyaan,autonomy):
    print(f'"{team_abhiyaan.data}: {autonomy.data}"')


def listener():

    rospy.init_node('listener_n', anonymous=True)

    # Appending two messages based on approximate time stamp method
    node_1 = message_filters.Subscriber('team_abhiyaan', String)
    node_2 = message_filters.Subscriber('autonomy', String)
    
    # Messages within 1 sec will get appended
    ts = message_filters.ApproximateTimeSynchronizer([node_1, node_2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

