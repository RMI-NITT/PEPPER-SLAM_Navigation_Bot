import rospy
import geometry_msgs.msg
import numpy as np
import time

cmd = geometry_msgs.msg.Twist()

pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
rospy.init_node('square', anonymous=True)
rate = rospy.Rate(50) # 50hz

speed=5.0

def forward():
    cmd.linear.x=0.1
    curr_time=time.time()
    while(time.time()-curr_time<=20.0):
        pub.publish(cmd)
        rate.sleep()
    cmd.linear.x=0.0
    pub.publish(cmd)

def left():
    cmd.angular.z=np.pi*0.02*speed
    curr_time=time.time()
    while(time.time()-curr_time<=24.033/speed):
        pub.publish(cmd)
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

def right():
    cmd.angular.z=-np.pi*0.02*speed
    curr_time=time.time()
    while(time.time()-curr_time<=24.033/speed):
        pub.publish(cmd)
        rate.sleep()
    cmd.angular.z=0.0
    pub.publish(cmd)

time.sleep(1)

if __name__ == '__main__':
    try:
        for i in range (0,3):
            forward()
            time.sleep(2)
            right()
            time.sleep(2)
        forward()
    except rospy.ROSInterruptException:
        pass
