#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import time
import math
from joystick.srv import message,messageResponse
from joystick.srv import route,routeResponse
# from joystick.msg import image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32
from geometry_msgs.msg import Pose

class Server:

	def __init__(self):
		rospy.init_node("server_node")
		service = rospy.Service("aruco",message,self.callback)
		service2=rospy.Service("director",route,self.callback2)
		self.bridge = CvBridge()
		# print("0")
		rospy.spin()
		
	def callback2(self,request ):
		print("hello")
		initial_coordinate_x,initial_coordinate_y = request.initial_pose.position.x,request.initial_pose.position.y
		#index=self.ros_ids.data.index(request.id)
		id_robot = request.id
		index=self.ids.index(int(id_robot.data))
		# print(index)
		# print(self.corners[index].shape)

		coordinates=[self.corners[index][0,0],self.corners[index][0,1],self.corners[index][0,2],self.corners[index][0,3]]
		final_coordinate_x1,final_coordinate_y1=coordinates[0] 
		final_coordinate_x2,final_coordinate_y2=coordinates[1]
		final_coordinate_x3,final_coordinate_y3=coordinates[2]
		final_coordinate_x4,final_coordinate_y4=coordinates[3]

		final_coordinate_x=(final_coordinate_x1+final_coordinate_x2+final_coordinate_x3+final_coordinate_x4)/4
		final_coordinate_y=(final_coordinate_y1+final_coordinate_y2+final_coordinate_y3+final_coordinate_y4)/4
		# print(final_coordinate_x)
		# print(final_coordinate_y)
		final_angle= np.arctan(final_coordinate_y-initial_coordinate_y)/(final_coordinate_x-initial_coordinate_x)
		print("byee")
		initial_angle = 2*(math.asin(request.initial_pose.orientation.w))
		# print(initial_angle)
		angle=final_angle-initial_angle
		print(final_angle)
		print(initial_angle)
		print(angle)
		distance = pow(pow((initial_coordinate_x-final_coordinate_x),2)+pow((initial_coordinate_y-final_coordinate_y),2),0.5)
		print(distance)
		response = routeResponse(distance=Float32(distance),angle=Float32(angle))

		return response


	def callback(self,request):
		
		# print("2")
		cv_image = self.bridge.imgmsg_to_cv2(request.aruco)

		ARUCO_DICT = {
			"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
			"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
			"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
			"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
			"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
			"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
			"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
			"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
			"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
			"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
			"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
			"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
			"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
			"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
			"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
			"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
			"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
			"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
			"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
			"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
			"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
			}

		aruco_type = "DICT_4x4_100"

		arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

		arucoParams = cv2.aruco.DetectorParameters()

		self.corners,ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
		
		if ids is None:
			ids = []
		ids=np.array(ids, dtype=np.float32)
		self.ids = tuple(ids.flatten().tolist())
		# print(self.ids)
	

		corners_flattened=np.array(self.corners)
		corners_flattened=corners_flattened.flatten()
		corners_flattened=corners_flattened.tolist()
		# print(corners_flattened)
		
		dim=[MultiArrayDimension() for i in range(3)]
		dim[0].size = len(ids)
		dim[1].size = 4
		dim[2].size = 2
		x = MultiArrayLayout(dim=dim,data_offset=0)
		ros_corners = Float32MultiArray(data=corners_flattened, layout=x)
		dim2=[MultiArrayDimension() for i in range(1)]
		dim2[0].size=1
		y=MultiArrayLayout(dim=dim2,data_offset=0)
		
		# print(ids)
		
		self.ros_ids=Float32MultiArray(data=ids,layout=y)
		response = messageResponse(corners = ros_corners, ids=self.ros_ids)
		# print(response)
			
		return response


		   
	



if __name__ == '__main__':
	print("3")
	server = Server()
	print("4")


	