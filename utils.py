# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2


# Define a Function to Read Object mapping from Text file
def read_mapping_of_objects():

	# Initialise Object Mapping Dictionary
	global object_mapping
	object_mapping = {}
	
	# Read Text File
	with open("object_mapping.txt") as file:
		
		# For every Line in File
		lines = file.readlines()
		for line in lines:

			# Extract Key and Value
			aruco_id, name = line.split(' ')

			# Form Object Mapping Dictionary
			object_mapping[int(aruco_id)] = name.replace('\n', '')
	
	# Return Object Mapping
	return object_mapping


# Define a Class to store the AruCo ID, Name, Pose of Object
class Object:

	# Define the Init Function
	def __init__(self, aruco_id, name):
				
		# Initialise the AruCo ID, Name, Pose, Final Pose of Object
		self.aruco_id = aruco_id
		self.name = name

		# If Object name is Origin
		if "Origin" in self.name:
			self.pose = np.identity(4)
			self.final_pose = np.identity(4)
		
		# Else if Object name is not Origin
		else:
			self.pose = None
			self.final_pose = None
		
		# Initialise arranged flag as None
		self.is_arranged = None
	

	# Define a Function to check if Final pose is close to Current pose
	def is_pose_final_pose(self):
		np.allclose(self.pose, self.final_pose, rtol = 0.01, atol = 0.01)


	# Define a Function to Display Class members
	def display(self):

		# Display Class Members
		print("AruCo ID: ", self.aruco_id)
		print("Name: ", self.name)
		print("Pose:\n", self.pose)
		print("Final_Pose:\n", self.final_pose)
		print("\n")
			

# Define a Class to store Multiple Objects
class Objects:

	# Define the Init Function
	def __init__(self, objects_with_aruco_ids):
		
		# Initialise Objects in Scene
		self.objects_with_aruco_ids = objects_with_aruco_ids
		self.objects = []
		for key, val in objects_with_aruco_ids.items():
			self.objects.append(Object(aruco_id = key, name = val))
	
	
	# Define a Function to get the Pose of Object using AruCo ID
	def get_value_of_object_using_key(self, aruco_id, key):

		# For every Object in Objects
		for object in self.objects:

			# If Object ID matches given AruCo ID
			if object.aruco_id == aruco_id:

				# If Key is Current Pose, Return Current Pose
				if key == "Pose":	return object.pose
				
				# If Key is Final Pose, Return Final Pose
				elif key == "Final_Pose":	return object.final_pose

	# Define a Function to Display all Objects
	def display(self):

		# For every Object
		for object in self.objects:

			# Display Data for that Object
			object.display()


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


# Define a Function to get the Pose of AruCo tag from List of detected AruCo tags wrt any Coordinate frame
def get_pose_of_aruco_tag(aruco_tags_data_wrt_frame, object_name):

	# Get the Mapping of Objects with AruCo IDs
	object_mapping = read_mapping_of_objects()

	# Get the AruCo ID corresponding to the Object name
	aruco_id = list(object_mapping.keys())[list(object_mapping.values()).index(object_name)]

	# For every AruCo tag detected in frame
	for aruco_tag_data_wrt_frame in aruco_tags_data_wrt_frame:

		# If AruCo ID matches given ID
		if aruco_id == aruco_tag_data_wrt_frame['ID']:

			# Return the Pose of that AruCo marker
			return np.array(aruco_tag_data_wrt_frame['Pose'])
	
	# Return None if not found
	return None
		

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


# Define a Function to Calculate Pose from Translation and Rotation vector/angles
def compute_pose_from_vectors_or_angles(translation, rotation, angle):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T

	# If Rotation is in Angles
	if angle:
		transformation_matrix[:3, :3] = R.from_euler('xyz', rotation, degrees = True).as_matrix()
	
	# If Rotation is in Vectors
	else:
		transformation_matrix[:3, :3] = cv2.Rodrigues(np.array(rotation))[0]
	transformation_matrix[3, 3] = 1

	# Return Transformation matrix
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to Get Translation and Rotation Quartenion/Angles from Pose
def get_translation_and_rotation_from_pose(pose, angle):

	# Round off Pose matrix and get Translation vector
	pose = round_matrix_list(pose, 3)
	translation = pose[:3, 3]

	# Get Rotation matrix
	rotation_matrix = pose[:3, :3]
	rotation = R.from_matrix(rotation_matrix)

	# If Rotation is in Angle
	if angle:
		rotation = rotation.as_euler('xyz', degrees = True)
	
	# If Rotation is in Quartenion
	else:
		rotation = rotation.as_quat()

	# Return Translation and Rotation
	return translation, rotation