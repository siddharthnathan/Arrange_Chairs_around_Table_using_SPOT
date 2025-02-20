# Import Necessary Libraries
import numpy as np
import cv2


# Define a Function to Read Camera Calibration parameters
def read_camera_calibration_params():

	# Read the Calibration parameters for both Cameras
	camera_calibration_params = {
									'Main_Camera': 
									{
										'Calibration_matrix': np.load('Camera_Calibration/Main_Camera/calibration_matrix.npy'),
										'Distortion_coefficients': np.load('Camera_Calibration/Main_Camera/distortion_coefficients.npy'),
									},
									'Side_Camera': 
									{
										'Calibration_matrix': np.load('Camera_Calibration/Side_Camera/calibration_matrix.npy'),
										'Distortion_coefficients': np.load('Camera_Calibration/Side_Camera/distortion_coefficients.npy'),
									}
								}
	
	# Return the Camera Calibration parameters
	return camera_calibration_params


# Define a Function to Resize Image frame for given Width
def resize_frame(frame, width):
	
	# Get the Shape of Image frame and maintain Aspect Ratio
	h, w, _ = frame.shape
	height = int(width * (h / w))

	# Resize Image frame and Return 
	frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_CUBIC)
	return frame