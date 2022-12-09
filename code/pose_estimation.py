#!/usr/bin/env python3

import cv2 # OpenCV library
import numpy as np
import yaml



# Calibration Data
CALIBRATION_FILE_PATH = '/home/esirem/Medhi_rob/robot_ws/src/calibrationdata/ost.yaml'
with open(CALIBRATION_FILE_PATH, "r") as data:
	try:
		calibration_data = yaml.safe_load(data)
	except yaml.YAMLError as exc:
		print(exc)


CAMERA_MATRIX = np.array(calibration_data["camera_matrix"]['data']).reshape(3,3)
DISTORTION_COEF = np.array(calibration_data["distortion_coefficients"]['data'])
HEIGHT = calibration_data["image_width"]
WIDTH = calibration_data["image_height"]

# Aruco Marker ID's
CURRENT_ID = 1
TARGET_ID = 10

MARKER_SIZE = 12 #centimeter

current_position = dict(corners=None, center=None, rvec=None, tvec=None) 
target_position = dict(corners=None, center=None, rvec=None, tvec=None)


def estimate_current_target_pose(frame, 
			current_id=CURRENT_ID,
			target_id=TARGET_ID, 
			matrix_coefficients=CAMERA_MATRIX, 
			distortion_coefficients=DISTORTION_COEF, 
			imshow=True):
			
	frame = frame.copy()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
	parameters = cv2.aruco.DetectorParameters_create()

	corners, ids_marker, rejected_img_points = cv2.aruco.detectMarkers(gray, marker_dict ,parameters=parameters)

	# If markers are detected
	if len(corners) > 0:
			rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
			corners, MARKER_SIZE, matrix_coefficients, distortion_coefficients)
			total_markers = range(0, ids_marker.size)
			for ids, corners, i in zip(ids_marker, corners, total_markers):
				cv2.polylines(
					frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
				)
				corners = corners.reshape(4, 2)
				corners = corners.astype(int)
				top_right = corners[0].ravel()
				top_left = corners[1].ravel()
				bottom_right = corners[2].ravel()
				bottom_left = corners[3].ravel()
	
				# Calculating the distance
				distance = np.sqrt(
					tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
				)
				# Draw the pose of the marker
				point = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rVec[i], tVec[i], 4, 4)
				cv2.putText(
					frame,
					f"id: {ids[0]} Dist: {round(distance, 2)}",
					top_right,
					cv2.FONT_HERSHEY_PLAIN,
					1.3,
					(0, 0, 255),
					2,
					cv2.LINE_AA,
				)
				cv2.putText(
					frame,
					f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
					bottom_right,
					cv2.FONT_HERSHEY_PLAIN,
					1.0,
					(0, 0, 255),
					2,
					cv2.LINE_AA,
				)
				print(f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",)
			
				if (ids[0] == current_id) and (len(corners) >= 1): 
					# get the center of the position by converting the four corners 
					current_position['center'] = [round(tVec[i][0][0],1) ,round(tVec[i][0][1],1)]
					current_position['corners'] = corners[0]       
					current_position['rvec'] = rVec[i]
					current_position['tvec'] = tVec[i]

				elif (ids[0] == target_id) and (len(corners) >= 1):
					# get the center of the position by converting the four corners
					target_position['center'] = [round(tVec[i][0][0],1) ,round(tVec[i][0][1],1)]
					target_position['corners'] = corners[0] 
					target_position['rvec'] = rVec[i]
					target_position['tvec'] = tVec[i]

	if imshow:
		cv2.imshow('Estimated Pose', frame)
		
		cv2.waitKey(1)& 0xFF == ord('q')

	return frame, current_position,target_position



