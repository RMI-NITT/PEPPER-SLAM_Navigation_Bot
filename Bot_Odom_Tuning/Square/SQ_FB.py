import rospy
import geometry_msgs.msg
import numpy as np
import time

from nav_msgs.msg import Odometry
import math

#Pub Code
cmd = geometry_msgs.msg.Twist()

pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
rospy.init_node('SQ_F', anonymous=True)
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
    
    th=th=math.degrees(2*math.atan2(Z,W))
    if th<0:
        th=th+360

rospy.Subscriber("/odom", Odometry, callback)

speed=5.0

def forward_x(goal):
    global x
    if goal>x:
        cmd.linear.x=0.01
        dist=goal-x
        tog=1
        tog2=1
        tog3=1
        tog4=1
        while(dist>=0.0):
            pub.publish(cmd)
            if(dist<=(goal-0.07) and tog3):
                cmd.linear.x=0.025
                tog3=0
            if(dist<=(goal-0.3) and tog4):
                cmd.linear.x=0.05
                tog4=0
            if(dist<=0.3 and tog):
                cmd.linear.x=0.025
                tog=0
            if(dist<=0.07 and tog2):
                cmd.linear.x=0.01
                tog2=0
            rate.sleep()
            dist=goal-x
    elif x>goal:
        cmd.linear.x=0.01
        dist=x-goal
        tog=1
        tog2=1
        tog3=1
        tog4=1
        while(dist>=0.0):
            pub.publish(cmd)
            if(dist<=(goal-0.07) and tog3):
                cmd.linear.x=0.025
                tog3=0
            if(dist<=(goal-0.3) and tog4):
                cmd.linear.x=0.05
                tog4=0
            if(dist<=0.3 and tog):
                cmd.linear.x=0.025
                tog=0
            if(dist<=0.07 and tog2):
                cmd.linear.x=0.01
                tog2=0
            rate.sleep()
            dist=x-goal
    cmd.linear.x=0.0
    pub.publish(cmd)
    time.sleep(2)

def forward_y(goal):
    global y
    if goal>y:
        cmd.linear.x=0.01
        dist=goal-y
        tog=1
        tog2=1
        tog3=1
        tog4=1
        while(dist>=0.0):
            pub.publish(cmd)
            if(dist<=(goal-0.07) and tog3):
                cmd.linear.x=0.025
                tog3=0
            if(dist<=(goal-0.3) and tog4):
                cmd.linear.x=0.05
                tog4=0
            if(dist<=0.3 and tog):
                cmd.linear.x=0.025
                tog=0
            if(dist<=0.07 and tog2):
                cmd.linear.x=0.01
                tog2=0
            rate.sleep()
            dist=goal-y
    elif y>goal:
        cmd.linear.x=0.01
        dist=y-goal
        tog=1
        tog2=1
        tog3=1
        tog4=1
        while(dist>=0.0):
            pub.publish(cmd)
            if(dist<=(goal-0.07) and tog3):
                cmd.linear.x=0.025
                tog3=0
            if(dist<=(goal-0.3) and tog4):
                cmd.linear.x=0.05
                tog4=0
            if(dist<=0.3 and tog):
                cmd.linear.x=0.025
                tog=0
            if(dist<=0.07 and tog2):
                cmd.linear.x=0.01
                tog2=0
            rate.sleep()
            dist=y-goal
    cmd.linear.x=0.0
    pub.publish(cmd)
    time.sleep(2)

def ang_diff(a,b):
    c=a-b
    if c<=-180:
        c=c+360
    return(c)

def right(tar_ang):
    global th
    if(ang_diff(tar_ang,th)<=0):
        cmd.angular.z=-np.pi*0.1
    tog=1
    while(ang_diff(tar_ang,th)<=0):
        pub.publish(cmd)
        if(ang_diff(tar_ang,th)>=-15.0):
            cmd.angular.z=np.pi*0.01
            tog=0
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)
    time.sleep(2)

def left(tar_ang):
    global th
    if(ang_diff(tar_ang,th)>=0):
        cmd.angular.z=np.pi*0.1
    tog=1
    while(ang_diff(tar_ang,th)>=0):
        pub.publish(cmd)
        if(ang_diff(tar_ang,th)<=15.0):
            cmd.angular.z=np.pi*0.01
            tog=0
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)
    time.sleep(2)

def square():
    forward_x(2.0)
    left(90.0)
    forward_y(2.0)
    left(180.0)
    forward_x(0.0)
    left(-90.0)
    forward_y(0.0)
    left(0.0)

if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass
