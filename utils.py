# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2


# Define a Function to Initialise Objects with their AruCo IDs and Poses
def initialise_objects_in_environment():
	objects_with_aruco_ids_and_poses = [
											{'ID': 1, 'Name': 'Left_Wall_1', 'Pose': None, 'Final_Pose': None}, 
											{'ID': 2, 'Name': 'Left_Wall_2', 'Pose': None, 'Final_Pose': None}, 
											{'ID': 3, 'Name': 'Origin', 'Pose': None, 'Final_Pose': None}, 
											{'ID': 4, 'Name': 'Right_Wall_2', 'Pose': None, 'Final_Pose': None}, 
											{'ID': 5, 'Name': 'Right_Wall_1', 'Pose': None, 'Final_Pose': None}, 
											{'ID': 6, 'Name': 'Chair_1', 'Pose': None, 'Final_Pose': None},
											{'ID': 7, 'Name': 'Chair_2', 'Pose': None, 'Final_Pose': None},
											{'ID': 8, 'Name': 'Chair_3', 'Pose': None, 'Final_Pose': None},
											{'ID': 9, 'Name': 'Chair_4', 'Pose': None, 'Final_Pose': None}
										]
	return objects_with_aruco_ids_and_poses
			

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


# Define a Function to set Objects with Corresponding AruCo tags
def get_object_with_aruco_tag(aruco_id):

	# For every Object in List
	for object in objects_with_aruco_ids:

		# If AruCo ID of object matches given ID
		if object['ID'] == aruco_id:

			# Return the Obejct name
			return object['Name']


# Define a Function to Calculate Pose from Translation and Rotation vector
def compute_pose_from_vectors(translation, rotation):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame and Return Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T
	transformation_matrix[:3, :3] = cv2.Rodrigues(np.array(rotation))[0]
	transformation_matrix[3, 3] = 1
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to Calculate Pose from Translation and Rotation angles
def compute_pose_from_angles(translation, rotation):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame and Return Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T
	transformation_matrix[:3, :3] = R.from_euler('xyz', rotation, degrees = True).as_matrix()
	transformation_matrix[3, 3] = 1
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to Calculate Pose from SPOT Data
def compute_pose_from_spot_data(spot_data):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Get the Translation vector & Rotation quartenion
	translation = spot_data.position
	translation = [translation.x, translation.y, translation.z]
	quartenion = spot_data.rotation
	quartenion = [quartenion.x, quartenion.y, quartenion.z, quartenion.w]

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


# Define a Function to Get Translation and Rotation Quartenion from Pose
def get_translation_and_quartenion_from_pose(pose):

	# Round off Pose matrix
	pose = round_matrix_list(pose, 3)

	# Get Translation vector
	translation = pose[:3, 3]

	# Get Quartenion
	rotation_matrix = pose[:3, :3]
	rotation = R.from_matrix(rotation_matrix)
	quaternion = rotation.as_quat()

	# Return Translation and Quartenion
	return translation, quaternion


# Define a Function to Get Translation and Rotation angles from Pose
def get_translation_and_rotation_from_pose(pose):

	# Round off Pose matrix
	pose = round_matrix_list(pose, 3)

	# Get Translation vector
	translation = pose[:3, 3]

	# Get Quartenion
	rotation_matrix = pose[:3, :3]
	rotation = R.from_matrix(rotation_matrix)
	euler_angles = rotation.as_euler('xyz', degrees=True)

	# Return Translation and Rotation angles
	return translation, euler_angles