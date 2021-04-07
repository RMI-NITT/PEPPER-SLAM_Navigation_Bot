import rospy
import geometry_msgs.msg
import numpy as np
import time

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
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

    X=data.pose.pose.orientation.x
    Y=data.pose.pose.orientation.y
    Z=data.pose.pose.orientation.z
    W=data.pose.pose.orientation.w

    x=data.pose.pose.position.x
    y=data.pose.pose.position.y
    #(X, Y, Z)=quaternion_to_euler(x, y, z, w)
    Z=euler_from_quaternion((X,Y,Z,W))
    
    th=math.degrees(Z[2])
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
            dif_ang=ang_diff(th,math.degrees(math.atan2(y1-y,goal-x))) #Calculates angle difference between angle to target point & bot facing angle
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

def ang_diff(a,b):
    c=a-b
    if c<=-180:
        c=c+360
    elif c>180:
        c=c-360
    return(c)


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

def forward():
    forward_x(2.90,0.0)

if __name__ == '__main__':
    try:
        forward()
    except rospy.ROSInterruptException:
        pass
