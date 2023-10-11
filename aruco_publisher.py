#!/usr//bin/env python3
import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


if __name__ == "__main__":
    rospy.init_node("image_publisher", anonymous=True)
    rospy.loginfo("Started Publishing")
    pub = rospy.Publisher('/image_topic', Image, queue_size=10)
    rate = rospy.Rate(10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        print(ret)
        if not ret:
            #print(frame)
            print("not connected")
        pub.publish(bridge.cv2_to_imgmsg(frame))
        rospy.loginfo("Publishing video")
        cv2.imshow('frame', frame)
        #cv2.waitKey(1)
        if cv2.waitKey(1) == ord('q'):
         break
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
