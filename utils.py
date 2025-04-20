# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2


# Define a Function to Round off values in a list
def round_float_list(float_list, num_decimal_places):
    
	# Return the List of Rounded off float values
    return [round(num, num_decimal_places) for num in float_list]


# Define a Function to Round off values in a Matrix
def round_matrix_list(matrix_list, num_decimal_places):

	# For every Row in Matrix
	for row in range(len(matrix_list)):

		# Update the List of Rounded off float values
		matrix_list[row] = [round(num, num_decimal_places) for num in matrix_list[row]]
	
	# Return the rounded off Rotation matrix
	return matrix_list


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


# Define a Function to set Objects with Corresponding AruCo tags
def get_object_with_aruco_tag(aruco_id):

	# Define the Objects with their AruCo IDs
	objects_with_aruco_ids = [
								{'ID': 4, 'Name': 'Main_Origin'}, 
						   		{'ID': 2, 'Name': 'Secondary_Origin'}, 
								{'ID': 5, 'Name': 'Chair'}
							 ]

	# For every Object in List
	for object in objects_with_aruco_ids:

		# If AruCo ID of object matches given ID
		if object['ID'] == aruco_id:

			# Return the Obejct name
			return object['Name']


# Define a Function to Calculate Pose from Translation and Rotation angles
def compute_pose_from_vectors(translation, rotation):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame and Return Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T
	transformation_matrix[:3, :3] = cv2.Rodrigues(np.array(rotation))[0]
	transformation_matrix[3, 3] = 1
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to Calculate Pose from Translation vector and Rotation quartenion
def compute_pose_from_quartenion(translation, quartenion):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame and Return Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T
	transformation_matrix[:3, :3] = R.from_quat(quartenion).as_matrix()
	transformation_matrix[3, 3] = 1
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to check if the given AruCo tag pose exists
def get_pose_of_aruco_tag(aruco_tags_data, aruco_tag_name):

	# For every AruCo tag data
	for aruco_tag_data in aruco_tags_data:

		# If the Name matches the required Tag name
		if aruco_tag_data['Name'] == aruco_tag_name:

			# Return the Pose of AruCo tag
			return np.array(aruco_tag_data['Pose'])
	
	# Return None if AruCo tag not found
	return None