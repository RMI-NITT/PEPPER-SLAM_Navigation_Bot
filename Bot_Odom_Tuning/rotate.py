import rospy
import geometry_msgs.msg
import numpy as np
import time

from nav_msgs.msg import Odometry
import math

#Pub Code
cmd = geometry_msgs.msg.Twist()

pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
rospy.init_node('rotu', anonymous=True)
rate = rospy.Rate(50) # 50hz

#Sub code

x=0.0
y=0.0
th=0.0
def callback(data):
    global x,y,th
    #rospy.loginfo(data.transforms.)

    Z=data.pose.pose.orientation.z
    W=data.pose.pose.orientation.w

    x=data.pose.pose.position.x
    y=data.pose.pose.position.y
    #(X, Y, Z)=quaternion_to_euler(x, y, z, w)
    #Z=euler_from_quaternion((X,Y,Z,W))
    
    th=math.degrees(2*math.atan2(Z,W))
    
    if th<0:
        th=th+360

rospy.Subscriber("/odom", Odometry, callback)

def ang_diff(a,b):
    c=a-b
    if c<=-180:
        c=c+360
    elif c>180:
        c=c-360
    return(c)

def right(tar_ang):
    global th
    if(ang_diff(tar_ang,th)<=0):
        cmd.angular.z=-np.pi*0.2
    tog=1
    while(ang_diff(tar_ang,th)<=0):
        pub.publish(cmd)
        if(ang_diff(tar_ang,th)>=-5.0):
            cmd.angular.z=np.pi*0.01
            tog=0
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

def left(tar_ang):
    global th
    if(ang_diff(tar_ang,th)>=0):
        cmd.angular.z=np.pi*0.2
    tog=1
    while(ang_diff(tar_ang,th)>=0):
        pub.publish(cmd)
        if(ang_diff(tar_ang,th)<=15.0):
            cmd.angular.z=np.pi*0.01
            tog=0
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

def left_fast(tar_ang):
    global th
    if(ang_diff(tar_ang,th)>=0):
        cmd.angular.z=np.pi*0.2
    while(ang_diff(tar_ang,th)>=0):
        pub.publish(cmd)
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

def rotate():
    for i in range(0,9):
            left_fast(180)
            left_fast(0)
    left_fast(180)
    left(0)

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass
