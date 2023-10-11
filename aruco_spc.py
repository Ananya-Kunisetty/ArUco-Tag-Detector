#!/usr//bin/env python3
import rospy
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import cv2
import time
from roboclaw_3 import Roboclaw
from joystick.srv import message,messageRequest
from joystick.srv import route,routeRequest
from geometry_msgs.msg import Pose

pub = rospy.Publisher('/drive_directives', String, queue_size=1)


class Client:
    def __init__(self):
        
        rospy.wait_for_service("director")
        rospy.wait_for_service("aruco")
        self.x=Int8()
        self.x.data=51
        self.rate=rospy.Rate(10)
        print("3")
        self.proxy=rospy.ServiceProxy("aruco", message)
        print("4")
        self.proxy2= rospy.ServiceProxy("director",route)   
        print("5")
        self.bridge = CvBridge()

    def callback(self, msg : Image):
            x=self.x
            
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            rospy.loginfo("Subscribing Video")
            print("1")

            try:
                image = self.bridge.cv2_to_imgmsg(cv_image)
                response=self.proxy(messageRequest(image))
                print(response)
        
                self.rate.sleep()

            except rospy.ServiceException as e:
                print("failed", e)

            corners=response.corners.data
            corners=np.asarray(corners)
            #print (corners)
            corners=corners.reshape(-1,4,2)
            ids=response.ids.data
            ids=np.asarray(ids)

            id_robot=50
            #print(type(ids[0]))
            #print(ids)
            index_robot=np.where(ids==50)[0][0]
            # print()

            #initialpose=corners[index_robot]
            pose_initial = Pose()
            pose_initial.position.x=(corners[index_robot][0][0]+corners[index_robot][1][0]+corners[index_robot][2][0]+corners[index_robot][3][0])/4
            pose_initial.position.y=(corners[index_robot][0][1]+corners[index_robot][1][1]+corners[index_robot][2][1]+corners[index_robot][3][1])/4
            pose_initial.orientation.w=math.atan((corners[index_robot][0][1]-corners[index_robot][1][1])/(corners[index_robot][0][0]-corners[index_robot][1][0]))
            pose_initial.orientation.w/=2
            # print((corners[index_robot][0][1]-corners[index_robot][1][1])/(corners[index_robot][0][0]-corners[index_robot][1][0]))
            # print(pose_initial.orientation.w)
            pose_initial.orientation.w= math.sin(pose_initial.orientation.w)
            try:
                            
                response2=self.proxy2(routeRequest(pose_initial, x))
                rospy.loginfo(response)
                print("works")

            except rospy.ServiceException as e:
                print("Service call failed %s",e)
                            
            if response2.distance.data == 0:
                if x<=52:
                   x=x+1
                
                    
            #need to write client which gives angle to be rotated and distance it has to move
            #message_client()
            
            message_publisher(response2.angle.data, response2.distance.data)
            
     

def message_publisher(angle,distance):
    if angle>0.50:
            dir="Left"
            print("left")
            pub.publish(dir)
    # else:
    #     dir="Stop"
    #     print("stop")
    # pub.publish(dir)

    elif angle<(-0.50):
                dir = "Right"
                print("right")
                pub.publish(dir)
    # else: 
    #     dir="Stop"
    #     print("stop1")
    # pub.publish(dir)  
    # angle<0.18 or angle>-0.18          
   
    else :
        if distance>10:
            dir="Forward"
            print("forward")
            pub.publish(dir)
        else: dir="Stop"
    pub.publish(dir)  

    
if __name__== "__main__":
    rospy.init_node('image_node',anonymous=True)
    roboclaw = Roboclaw("/dev/drive",9600)
    roboclaw.Open()
    rospy.loginfo("Started Subscribing")

    client=Client()
    sub = rospy.Subscriber('/image_topic',Image,client.callback)

    rospy.spin()