#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32

from thrust_allocation.msg import pwm
from sensor_msgs.msg import Joy
x_flag = 0
pub = rospy.Publisher('/mallard/cmd_vel',Twist, queue_size = 10)

def callback(twist):
    global x_flag 
    if (x_flag == 1):
        # value between 0 to 1 corresponds to 0-400 pwm 
        twist.linear.x = 1
    else:
        twist.linear.x = 0
    # unchanged:    
    # twist.linear.y
    # twist.angular.z
    pub.publish(twist)

def callback_joy(data):
    global x_flag
    twist = Twist()
    # twist.linear.x = 1*data.axes[1] 

    x_button = data.buttons[0] #on PS4 corresponds to "x" button 

    if (x_button == 1):
        #set flag
        x_flag = 1
    else:
        x_flag = 0
    # pub.publish(twist)

def velocity_allocation  ():
    rospy.init_node('controller_disengage') #, anonymous=True)
    rospy.Subscriber('/cmd_controller_konrad', Twist, callback)
    rospy.Subscriber('/joy',Joy,callback_joy)

    rospy.spin()
    # rospy.spin() simply keeps your node from exiting 
    # until the node has been shutdown.

if __name__=='__main__':
    velocity_allocation()