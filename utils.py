# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
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
		self.pose = None

		# If Object name is Origin
		if "Origin" in self.name:
			self.final_pose = np.identity(4)
		
		# Else if Object name is not Origin
		else:
			self.final_pose = None
		

	# Define a Function to check if Current pose is close to Final pose
	def is_pose_at_final_pose(self):

		# Calculate the Frobenius norm of the difference
		frobenius_distance = np.linalg.norm(self.pose - self.final_pose, 'fro')

		# If Frobenius norm of the difference exceeds threshold limit, assign False
		if frobenius_distance > 0.5:
			return False

		# Else, Return True
		return True


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
		self.chairs = []
		self.walls = []
		self.waypoints = []

		# Define Range scale wrt Distance between Chairs
		self.scale_of_range = 3

		# Define the Number of Waypoints and Angle between Waypoints in Scene
		self.num_of_waypoints = 6
		self.angle_bw_waypoints = 2 * np.pi / self.num_of_waypoints
		
		# For every Object with ArUco IDs
		for key, val in objects_with_aruco_ids.items():

			# If Object is Chair
			if "Chair" in val:
				self.chairs.append(Object(aruco_id = key, name = val))
			
			# Else if Object is not Chair (AruCo on Walls)
			else:
				self.walls.append(Object(aruco_id = key, name = val))
					
	
	# Define a Function to Compute the Pose of Table using Final pose of Chairs
	def compute_pose_of_table(self):

		# Initialise Table Object
		self.table = Object(aruco_id = None, name = "Table")

		# Initialise Translation and Rotation arrays of Chairs
		translation_of_chairs = []
		rotation_of_chairs = []

		# For every Chair object
		for chair in self.chairs:

			# Store the Translation and Rotation components of Chairs
			translation, rotation = get_translation_and_rotation_from_pose(chair.final_pose, angle = True)
			translation_of_chairs.append(translation)
			rotation_of_chairs.append(rotation)

		# Compute the Translation and Rotation component of Table by taking Mean
		translation_of_table = np.mean(translation_of_chairs, axis = 0)
		rotation_of_table = np.mean(rotation_of_chairs, axis = 0)
		rotation_of_table = [0, rotation_of_table[1], 0]

		# Compute the Pose of Table
		self.table.final_pose = compute_pose_from_vectors_or_angles(translation_of_table, rotation_of_table, angle = True)

		# Return the Updated object with Table data
		return self
	
	
	# Define a Function to Compute the Pose of Waypoints using Pose of Table
	def compute_poses_of_waypoints(self):

		# Retrieve the Poses of Chairs in scene
		pose_of_chair_1, pose_of_chair_2 = self.get_pose_of_chair("Chair_1"), self.get_pose_of_chair("Chair_2")
		pose_of_chair_3, pose_of_chair_4 = self.get_pose_of_chair("Chair_3"), self.get_pose_of_chair("Chair_4")
		
		# Compute the Average Distance between Chairs
		distance_bw_chairs = round(np.mean([compute_distance_between_poses(pose_of_chair_1, pose_of_chair_2), 
									  		compute_distance_between_poses(pose_of_chair_3, pose_of_chair_4)]), 3)

		# Create Object for Waypoint 1 (South of Table)
		waypoint_1 = Object(aruco_id = None, name = "Waypoint_1")

		# Determine Pose of Waypoint 1 using Pose of Table and Scale Range around Table wrt SPOT body frame
		waypoint_1.final_pose = self.table.final_pose @ compute_pose_from_vectors_or_angles(translation = [0, 0, self.scale_of_range * distance_bw_chairs],
														   		 				   			rotation = [0, 90, 90], angle = True)
		self.waypoints.append(waypoint_1)

		# For every remaining Waypoints
		for i in range(self.num_of_waypoints - 1):

			# Create Object for Waypoint
			waypoint = Object(aruco_id = None, name = "Waypoint_" + str(i+2))

			# Determine the Pose of Waypoint using Pose of Table, Scale Range around Table and Angle inscribed around Table
			waypoint.final_pose = self.compute_pose_of_waypoint(waypoint_1, angle = self.angle_bw_waypoints * (i+1))

			# Append Waypoint data
			self.waypoints.append(waypoint)

		# Return the Updated object with Waypoints data
		return self


	# Define a Function to Determine the Pose of Waypoint using Pose of Table, Scale Range around Table and Angle inscribed around Table wrt SPOT body frame
	def compute_pose_of_waypoint(self, waypoint, angle):

		# Get the Pose of Table and Waypoint
		pose_of_table, pose_of_waypoint = self.table.final_pose, waypoint.final_pose
		
		# Compute the Relative pose of Waypoint wrt Table
		relative_pose_of_waypoint = np.linalg.inv(pose_of_table) @ pose_of_waypoint
		distance_of_relative_pose = compute_distance_from_pose(relative_pose_of_waypoint)
		
		# Compute the Pose of Waypoint and Return
		s, c = math.sin(angle), math.cos(angle)
		return round_matrix_list(pose_of_table @ compute_pose_from_vectors_or_angles(translation = [-s * distance_of_relative_pose, 0, c * distance_of_relative_pose],
														 	   				   		 rotation = [0, 90-angle, 90], angle = True) @ np.array([
																																				[ c,  s,  0,  0],
																																				[-s,  c,  0,  0],
																																				[ 0,  0,  1,  0],
																																				[ 0,  0,  0,  1]
																					 														]), 3)


	# Define a Function to get the Pose of Wall using Name
	def get_pose_of_wall(self, name):

		# For every Wall Object
		for object in self.walls:

			# If Object Name matches given Name
			if object.name == name:
				
				# Return Final Pose
				return object.final_pose
	

	# Define a Function to get the Pose of Chair using Name
	def get_pose_of_chair(self, name):

		# For every Chair Object
		for object in self.chairs:

			# If Object Name matches given Name
			if object.name == name:
				
				# Return Final Pose
				return object.final_pose
	
	
	# Define a Function to check if Chairs are Arranged around Table
	def is_chairs_arranged(self):

		# For every Chair
		for object in self.chairs:

			# If Chair is not Arranged
			if not object.is_pose_at_final_pose():

				# Return False
				return False
		
		# Else Return True when its Arranged
		return True
	

	# Define a Function to Display all Objects
	def display(self):
		
		# Display Data for every Wall
		print("----- ArUco on Walls -----\n")
		for object in self.walls:
			object.display()
		
		# Display Data for every Chair
		print("----- ArUco on Chairs -----\n")
		for object in self.chairs:
			object.display()
		
		# Display Data for Table
		print("----- Pose of Table -----\n")
		self.table.display()

		# Display Data for Waypoints
		print("----- Pose of Waypoints -----\n")
		for object in self.waypoints:
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

	# If there are AruCo tags detected in the Frame
	if aruco_tags_data_wrt_frame is not None:

		# For every AruCo tag detected in frame
		for aruco_tag_data_wrt_frame in aruco_tags_data_wrt_frame:

			# If AruCo ID matches given ID
			if aruco_id == aruco_tag_data_wrt_frame['ID']:

				# Return the Pose of that AruCo marker
				return aruco_tag_data_wrt_frame['Pose']
	
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


# Define a Function to Compute the Absolute Distance from Relative Pose
def compute_distance_from_pose(pose):

	# Extract the Translation vector
	translation = [pose[0][3], pose[1][3], pose[2][3]]

	# Compute and Return the Distance
	distance = np.linalg.norm(translation)
	return round(distance, 3)


# Define a Function to Compute the Distance a Pose wrt Reference Pose
def compute_distance_between_poses(pose, reference_pose):

	# Compute the Relative pose
	relative_pose = round_matrix_list(np.linalg.inv(reference_pose) @ pose, 3)

	# Return the Distance of Pose
	return compute_distance_from_pose(relative_pose)