#! /usr/bin/env python3
import yaml
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from planner.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge
import cv2.aruco as aruco
br = CvBridge()
first = False

class Img:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		#cv2.namedWindow("image1", 1)
		#[self.matrix_coefficients, self.distortion_coefficients] = load_coefficients((3,3))
		self.matrix_coefficients = np.zeros((3,3))
		self.distortion_coefficients = np.zeros((1,5))
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.image_callback,queue_size = 1) #/compressed
		# topic name for usb_cam is : camera1/usb_cam1/image_raw/compressed
		# print("Img before bb_pub")
		self.bb_pub = rospy.Publisher('/bb_aruco', BoundingBoxes, queue_size=10)
		self.bboxes = BoundingBoxes()

		# self.frame = np.zeros((480,640,3))
		# print("Img after bb_pub")
	'''def load_coefficients(mat_shape):
		camera_matrix = np.zeros((3,3))
		dist_matrix = np.zeros((1,5))
		return [camera_matrix, dist_matrix]
	'''
    	#""" Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    #cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    #camera_matrix = cv_file.getNode("K").mat()
    #dist_matrix = cv_file.getNode("D").mat()

    #cv_file.release()

	def image_callback(self,msg):
		global first
		#self.frame = br.compressed_imgmsg_to_cv2(msg)
		self.frame = br.imgmsg_to_cv2(msg)
		first = True
		# print(self.frame.shape)

	def publish_bb(self):
		frame = self.frame
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) #Supposed to be 5x5_250
		parameters = aruco.DetectorParameters_create()
		# print("At Image Callback")
		corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters = parameters, cameraMatrix = self.matrix_coefficients, distCoeff = self.distortion_coefficients) #-- works as initial parameters probably - look to line 15,16
		if np.all(ids is not None):
			for i in range(0, len(ids)):
				rvec,tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i],0.02, self.matrix_coefficients, self.distortion_coefficients)
				(rvec - tvec).any()
			aruco.drawDetectedMarkers(frame, corners)
			aruco.drawAxis(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)

			self.bboxes = BoundingBoxes()

		for i in range(len(corners)):	
			msg = BoundingBox()
			msg.xmin = int(corners[i][0][0][0])
			msg.ymin = int(corners[i][0][0][1])
			msg.xmax = int(corners[i][0][1][0])
			msg.ymax = int(corners[i][0][1][1])
			msg.id = ids[i][0]
			msg.x,msg.y,channel=frame.shape
			self.bboxes.bounding_boxes.append(msg)
			print("Detected: ", abs((msg.xmin - msg.xmax)*(msg.ymin - msg.ymax)))	
				
		if not len(corners):
			msg = BoundingBoxes()
			msg.bounding_boxes=[]
			self.bboxes.bounding_boxes=[]
			print("Not Detected.")
			
		self.bb_pub.publish(self.bboxes)
		print(corners)

		print(frame.shape)
		frame = cv2.circle(frame, (320,240), 5, (0, 0, 255), -1)

		

		cv2.imshow('image1',frame)
		key = cv2.waitKey(3)

	def main(self):
		while not rospy.is_shutdown():
			self.publish_bb()
			rospy.sleep(0.1)

if __name__ == '__main__':
	rospy.init_node('aruco_pub')
	server = Img()
	while not first and not rospy.is_shutdown():
		pass
	if first:
		server.main()
	rospy.spin()