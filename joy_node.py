#!/usr//bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32,Float32MultiArray,String

drive_pub = rospy.Publisher('/drive_directives', String, queue_size=1)

def joy_callback(joy_inp):
    dir = String()

    joy_inp_axes = joy_inp.axes
    if joy_inp_axes[1]==-1:
        print("Forward")
        dir.data = "Forward"
    elif joy_inp_axes[1]==1:
        print("Backward")
        dir.data = "Backward"
    elif joy_inp_axes[0]==1:
        print("Right")
        dir.data = "Right"
    elif joy_inp_axes[0]==-1:
        print("Left")
        dir.data = "Left"
    else :
        print("Stop")
        dir.data = "Stop"
    drive_pub.publish(dir)

if __name__=='__main__':
    rospy.init_node("joystick")
    rospy.loginfo("Starting base Station Joystick Node")
    rospy.Subscriber("/joy",Joy,joy_callback)
    rospy.spin()
