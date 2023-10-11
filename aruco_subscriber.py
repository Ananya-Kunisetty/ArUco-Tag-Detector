#!/usr//bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from roboclaw_3 import Roboclaw
from joystick.srv import message,messageRequest
from joystick.srv import route,routeRequest
from geometry_msgs.msg import Pose

pub = rospy.Publisher('/drive_directives', String, queue_size=1)

def message_client(cv_image):
    rospy.wait_for_service("aruco")
    proxy=rospy.Service("aruco",route)
    while not rospy.is_shutdown(): 
        try:
            response=proxy("")

    rospy.wait_for_service("director")
    bridge = CvBridge()
    rate=rospy.Rate(1)
    proxy = rospy.ServiceProxy("director",message)

    while not rospy.is_shutdown():
            try:
                 ros_imgmsg = bridge.cv2_to_imgmsg(cv_image)
                 response=proxy("initial_pose,id",route)

            except rospy.ServiceException as e:
                print("Service call failed %s",e)
                 
    return response

def message_publisher(angle,distance):
    if angle>0:
        while angle>0:
            dir="Left"
            pub.publish(dir)
    dir="Stop"
    pub.publish(dir)

    if angle<0:
            while angle<0:
                dir = "Right"
                pub.publish(dir)
    dir="Stop"
    pub.publish(dir)            
   
    while distance>0:
         dir="Forward"
         pub.publish(dir)
    dir="Stop"
    pub.publish(dir)  



def callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg)
    rospy.loginfo("Subscribing Video")
    #need to write client which gives angle to be rotated and distance it has to move
    
    
        
    response = message_client(cv_image)
    message_publisher(response)
    



    
if __name__== "__main__":
    rospy.init_node('/image_topic',anonymous=True)
    roboclaw = Roboclaw("/dev/drive",9600)
    roboclaw.Open()
    rospy.loginfo("Started Subscribing")
    sub = rospy.Subscriber('/image_topic',String,callback)
    rospy.spin()