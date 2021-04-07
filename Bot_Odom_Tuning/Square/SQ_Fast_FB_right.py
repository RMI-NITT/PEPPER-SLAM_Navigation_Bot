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
    
    th=math.degrees(2*math.atan2(Z,W))

    if th<0:
        th=th+360

rospy.Subscriber("/odom", Odometry, callback)

def forward_x(goal,y1):
    global x,y,th
    if goal>x:
        cmd.linear.x=0.1
        dist=goal-x
        while(dist>=0.0):
            pub.publish(cmd)
            #print(math.degrees(math.atan2(y1-y,goal-x)))
            dif_ang=ang_diff(th,math.degrees(math.atan2(y1-y,goal-x)))
            if(dif_ang>0):
                cmd.angular.z=-0.01
            elif(dif_ang<0):
                cmd.angular.z=0.01
            else:
                cmd.angular.z=0.0
            rate.sleep()
            dist=goal-x
    elif x>goal:
        start=x
        cmd.linear.x=0.1
        dist=x-goal
        while(dist>=0.0):
            pub.publish(cmd)
            dif_ang=ang_diff(th,math.degrees(math.atan2(y1-y,goal-x)))
            if(dif_ang>0):
                cmd.angular.z=-0.01
            elif(dif_ang<0):
                cmd.angular.z=0.01
            else:
                cmd.angular.z=0.0
            rate.sleep()
            dist=x-goal
    cmd.linear.x=0.0
    cmd.angular.z=0.0
    pub.publish(cmd)

def forward_y(x1,goal):
    global x,y,th
    if goal>y:
        cmd.linear.x=0.1
        dist=goal-y
        while(dist>=0.0):
            pub.publish(cmd)
            dif_ang=ang_diff(th,math.degrees(math.atan2(goal-y,x1-x)))
            if(dif_ang>0):
                cmd.angular.z=-0.01
            elif(dif_ang<0):
                cmd.angular.z=0.01
            else:
                cmd.angular.z=0.0
            rate.sleep()
            dist=goal-y
    elif y>goal:
        start=y
        cmd.linear.x=0.1
        dist=y-goal
        while(dist>=0.0):
            pub.publish(cmd)
            dif_ang=ang_diff(th,math.degrees(math.atan2(goal-y,x1-x)))
            if(dif_ang>0):
                cmd.angular.z=-0.01
            elif(dif_ang<0):
                cmd.angular.z=0.01
            else:
                cmd.angular.z=0.0
            rate.sleep()
            dist=y-goal
    cmd.linear.x=0.0
    cmd.angular.z=0.0
    pub.publish(cmd)

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
        if(ang_diff(tar_ang,th)>=-7.0):
            cmd.angular.z=-np.pi*0.01
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
        if(ang_diff(tar_ang,th)<=7.0):
            cmd.angular.z=np.pi*0.01
            tog=0
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

def square():
    forward_x(2.0,0.0)
    right(-90.0)
    forward_y(2.0,-2.0)
    right(180.0)
    forward_x(0.0,-2.0)
    right(90.0)
    forward_y(0.0,0.0)
    right(0.0)

if __name__ == '__main__':
    try:
        square()
        #right(-90.0)
    except rospy.ROSInterruptException:
        pass
