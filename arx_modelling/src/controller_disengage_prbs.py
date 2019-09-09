#!/usr/bin/env python
import rospy
import sys
from   prbs import prbs_sequence

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32

from thrust_allocation.msg import pwm
from sensor_msgs.msg import Joy

x_flag = 0
#prbs_sequence(N-number of steps,band-length of constant signal after transition, range-y values)
prbs_seq = prbs_sequence(100,10,1) #initialize the prbs signal
a = [] #intialize to empty list; it stores prbs signal that is sequentially shortened

pub = rospy.Publisher('/mallard/cmd_vel',Twist, queue_size = 10)
pub_prbs = rospy.Publisher('mallard/prbs_input',Twist,queue_size = 10)


def callback(twist):
    global x_flag,a

    if (x_flag == 1):
        # value between 0 to 1 corresponds to 0-400 pwm 
        if (a): 
            #pops first element and removes it from the list
            twist.linear.x = a.pop(0)
            pub_prbs.publish(twist)
        else:
            twist.linear.x = 0 #list is empty

    else:
        twist.linear.x = 0
    # unchanged:    
    # twist.linear.y
    # twist.angular.z
    pub.publish(twist)

def callback_joy(data):
    global x_flag,a
    x_button = data.buttons[0] #on PS4 corresponds to "x" button 
    
    if (x_button == 1):
        if (not a): #true only if a list is empty
            a = prbs_seq[:] #set a list with entire pwm signal
        #set flag
        x_flag = 1
    else:
        if (a): #true if elements in the list
            a = prbs_seq[:] #restore a list
        x_flag = 0

def velocity_allocation  ():
    rospy.init_node('controller_disengage') #, anonymous=True)
    rospy.Subscriber('/cmd_controller_konrad', Twist, callback)
    rospy.Subscriber('/joy',Joy,callback_joy)
    rospy.spin()
    # rospy.spin() simply keeps your node from exiting 
    # until the node has been shutdown.

if __name__=='__main__':
    global prbs_seq
    velocity_allocation()