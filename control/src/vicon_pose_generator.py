#!/usr/bin/env python

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped

initial_x_vicon = 0
initial_y_vicon = 0
initial_q_vicon = [0,0,0,0]
flag = True


def vicon_callback(vicon_data):
    global initial_x_vicon,initial_y_vicon,initial_q_vicon,flag

    t = vicon_data
    p = PoseStamped()
    # q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    # # for first run get current position and load first goals:
    # if flag_first:
    #     x0 = data.pose.position.x
    #     y0 = data.pose.position.y

    x_vicon = t.transform.translation.x
    y_vicon = t.transform.translation.y
    q_vicon = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

    # INITIALIZE
    if (initial_x_vicon == 0 and flag == True):
        initial_x_vicon = x_vicon
        initial_y_vicon = y_vicon
        initial_q_vicon = q_vicon
        initial_q_vicon[3] = -initial_q_vicon[3] #negate for inverse
        flag = False

    # Quaternion difference (offset by initial value)
    q_result =  tft.quaternion_multiply(q_vicon,initial_q_vicon)

    # Write to PoseStamp positions:
    p.header.stamp= rospy.Time.now()
    p.pose.position.x  = x_vicon - initial_x_vicon
    p.pose.position.y  = y_vicon - initial_y_vicon
    # PoseStamped orientations:
    p.pose.orientation.x = q_result[0]
    p.pose.orientation.y = q_result[1]
    p.pose.orientation.z = q_result[2]
    p.pose.orientation.w = q_result[3]
    
    pub_vicon_pose.publish(p)



if __name__ == '__main__':
    rospy.init_node('vicon_pose_generator', anonymous=True)

    # SUBSRCIBER
    rospy.Subscriber("/vicon/Mallard/Mallard",TransformStamped,vicon_callback)
    #  PUBLISHER
    pub_vicon_pose = rospy.Publisher("/vicon_pose",PoseStamped,queue_size=10)

    rospy.spin()