import rospy
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
import math

"""
def quaternion_to_euler(x, y, z, w):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z
"""
    

    

def callback(data):
    #rospy.loginfo(data.transforms.)

    #x=data.pose.pose.orientation.x
    #y=data.pose.pose.orientation.y
    z=data.pose.pose.orientation.z
    w=data.pose.pose.orientation.w

    #(X, Y, Z)=quaternion_to_euler(x, y, z, w)
    
    #Z=euler_from_quaternion((x,y,z,w))
    th=math.degrees(2*math.atan2(z,w))
    
    if th<0:
        th=th+360

    print('\nEuler Pos')
    print('x: ' + str(data.pose.pose.position.x))
    print('y: ' + str(data.pose.pose.position.y))
    print('θ: ' + str(th)+'°')

def listener():

     # In ROS, nodes are uniquely named. If two nodes with the same
     # name are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
     rospy.init_node('qt2euler', anonymous=True)
 
     rospy.Subscriber("/odom", Odometry, callback)
 
     # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()
 
if __name__ == '__main__':
     listener()
