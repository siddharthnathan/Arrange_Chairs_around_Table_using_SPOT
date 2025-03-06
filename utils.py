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
	objects_with_aruco_ids = [{'ID': 24, 'Name': 'Chair'}, {'ID': 19, 'Name': 'SPOT'}]

	# For every Object in List
	for object in objects_with_aruco_ids:

		# If AruCo ID of object matches given ID
		if object['ID'] == aruco_id:

			# Return the Obejct name
			return object['Name']
	

# Define a Function to define size of AruCo markers
def get_size_of_aruco_tag(aruco_id):

	# Define Size of AruCo markers in m
	aruco_sizes = [{'ID': 24, 'Size': 7.75 * 2.5/100}, {'ID': 19, 'Size': 7 * 2.5/100}, {'ID': 32, 'Size': 0.1}]

	# For every AruCo tag in sizes list
	for aruco_size in aruco_sizes:

		# Check if their IDs match
		if int(aruco_size['ID']) == aruco_id:
	
			# Return the Aruco tag Dictionary
			return aruco_size['Size']


# Define a Function to convert Rotation vector to Rotation angles in degrees
def convert_rotation_vector_to_angles(rotation_vector):
	
	# Convert rotation vector to a Rotation object
	rotation = R.from_rotvec(rotation_vector)

	# Convert to Euler angles in degrees
	euler_angles = rotation.as_euler('xyz', degrees = True)

	# Return the rounded off angles
	return round_float_list(euler_angles, 3)


# Define a Function to get Rotation matrix from Rotation angles in Degrees
def get_rotation_matrix_from_angles(angles):
  
	# Compute Rotation matrix from Rotation euler angles
	rotation_matrix = R.from_euler('xyz', angles, degrees = True).as_matrix()
	rotation_matrix = round_matrix_list(rotation_matrix, 4)

	# Return the Rotation matrix
	return rotation_matrix