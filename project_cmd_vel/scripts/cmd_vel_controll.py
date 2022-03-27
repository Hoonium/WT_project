#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import math

rospy.init_node("cmd_vel_controll")
robot_size_x = rospy.get_param('~robot_size_X')
robot_size_y = rospy.get_param('~robot_size_Y')
safe_zone = rospy.get_param("~safe_zone")


robot_cmd_pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=1)

pre_scan_time = 0
usr_cmd_data = Twist()

def usr_cmd_CB(data) :
    global usr_cmd_data
    usr_cmd_data = data

def scan_CB(data) :
    global init_scan_hz_flag
    global pre_scan_time
    if not pre_scan_time :
        pre_scan_time = rospy.Time.now().secs + (rospy.Time.now().nsecs / 1000000000.0)
        init_scan_hz_flag = True
        return 0
    scan_time = rospy.Time.now().secs + (rospy.Time.now().nsecs / 1000000000.0)
    scan_rate = scan_time - pre_scan_time
    pre_scan_time = scan_time

    for i in range(len(data.ranges)) :
        if data.ranges[i] != float("inf") :
            scan_x = math.cos(math.radians(i)) * data.ranges[i]
            scan_y = math.sin(math.radians(i)) * data.ranges[i]
            
            _scan_x = scan_x*math.cos(usr_cmd_data.angular.z*scan_rate) + scan_y*math.sin(usr_cmd_data.angular.z*scan_rate) + (usr_cmd_data.linear.y * scan_rate)
            _scan_y = scan_y*math.cos(usr_cmd_data.angular.z*scan_rate) - scan_x*math.sin(usr_cmd_data.angular.z*scan_rate) + (usr_cmd_data.linear.x * scan_rate)

            if safe_list[int(round(math.degrees(math.atan2(_scan_y,_scan_x))))] >= _scan_x / math.cos(math.atan2(_scan_y,_scan_x)) :
                if _scan_x / math.cos(math.atan2(_scan_y,_scan_x)) >= data.ranges[int(round(math.degrees(math.atan2(_scan_y,_scan_x))))] or data.ranges[int(round(math.degrees(math.atan2(_scan_y,_scan_x))))] == float("inf"):
                    print("continue")
                    continue
                robot_cmd_pub.publish(Twist())
                return 0

    robot_cmd_pub.publish(usr_cmd_data)


def makeSafezone():
    scan_data = rospy.wait_for_message("/scan",LaserScan,1)
    scan_data_ranges_len = len(scan_data.ranges)
    
    right_thd, left_thd = (-robot_size_x/2 - base_scan_x) - safe_zone, (robot_size_x/2 - base_scan_x) + safe_zone
    back_thd, front_thd = (robot_size_y/2 - base_scan_y) + safe_zone, (-robot_size_y/2 - base_scan_y) - safe_zone

    theta1 = int(math.degrees(math.atan2(back_thd,left_thd)))
    theta2 = int(math.degrees(math.atan2(back_thd,right_thd)))
    theta3 = int(math.degrees(math.atan2(front_thd,right_thd)))
    theta4 = int(math.degrees(math.atan2(front_thd,left_thd)))
    
    #left [theta4:],[0:theta1] back [theta1:theta2] right [theta2:theta3] front [theta3:theta4]
    print(front_thd,"\n",back_thd,"\n",left_thd,"\n",right_thd,"\n")
    print(theta1,"\n",theta2,"\n",theta3,"\n",theta4,"\n")
    safe_list = [0] * scan_data_ranges_len

    for i in range(theta1) : # L
        safe_list[i] =  (left_thd / 1000) / math.cos(math.radians(i))
    for i in range(theta1,theta2) : # B
        safe_list[i] =  (back_thd / 1000) / math.sin(math.radians(i))
    for i in range(theta2,360+theta3) : # R
        safe_list[i] =  (right_thd / 1000) / math.cos(math.radians(i))
    for i in range(360+theta3,360+theta4) : # F
        safe_list[i] =  (front_thd / 1000) / math.sin(math.radians(i))
    for i in range(360+theta4,360): # L
        safe_list[i] =  (left_thd / 1000) / math.cos(math.radians(i))

    return safe_list
    

if __name__ == '__main__' :
    usr_cmd_vel_sub = rospy.Subscriber("/usr/cmd_vel",Twist,usr_cmd_CB,queue_size=1)
    scan_sub = rospy.Subscriber("/scan",LaserScan,scan_CB,queue_size=1)

    tf_static = rospy.wait_for_message("/tf_static",TFMessage,1)

    for i in tf_static.transforms :
        if i.child_frame_id == "base_scan" :
            base_scan_x, base_scan_y = i.transform.translation.x, i.transform.translation.y
            break

    safe_list = makeSafezone()

    rospy.spin()