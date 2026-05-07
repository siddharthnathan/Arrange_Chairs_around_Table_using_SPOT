# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import json
import cv2


# Define a Class to store the AruCo ID, Name, Pose of Object
class Object:

	# Define the Init Function
	def __init__(self, aruco_id, name, translation, rotation):
				
		# Initialise the AruCo ID, Name
		self.aruco_id = aruco_id
		self.name = name
		
		# Initialise the Translation and Rotation components of Final pose
		self.final_pose = {'Translation': translation, 'Rotation': rotation, 'Pose': compute_pose_from_components(translation, rotation)}

		# Initialise the Translation and Rotation components of Current pose
		self.pose = {'Translation': None, 'Rotation': None, 'Pose': None}

		# If name is Chair
		if "Chair" in self.name:

			# Initialise the Translation and Rotation components of Final pose wrt Camera frame
			self.final_pose_wrt_camera = {'Translation': None, 'Rotation': None, 'Pose': None}

			# Initialise the Translation and Rotation components of Current pose wrt Camera frame
			self.pose_wrt_camera = {'Translation': None, 'Rotation': None, 'Pose': None}

			# Initialise Displacement from Final pose
			self.displacement = None

			# Initialise Translation and Rotation threshold to check Displacement
			self.translation_threshold = 0.1
			self.rotation_threshold = 10


	# Define a Function to check if Current pose is close to Final pose
	def is_pose_at_final_pose(self):

		# Get the Displacement matrix of Chair wrt Final pose
		self.displacement = np.linalg.inv(self.final_pose_wrt_camera['Pose']) @ self.pose_wrt_camera['Pose']

		# Extract the Displacement Translation and Rotation components of Chair
		translation, rotation = get_components_from_pose_for_chair(self.displacement)
		x, _, z = translation
		_, ry, _ = rotation
		self.displacement = [x, z, ry]

		# If the Difference exceeds a threshold, Return False
		if abs(x) > self.translation_threshold \
		or abs(z) > self.translation_threshold \
		or abs(ry) > self.rotation_threshold \
      	and abs(ry) < 20:
			return False

		# Else, Return True
		return True


	# Define a Function to Display Class members
	def display(self):

		# Display Class Members
		print("AruCo ID: ", self.aruco_id)
		print("Name: ", self.name)
		
		# Display Translation and Rotation if Current pose is not None
		if self.pose['Pose'] is not None:
			print("Pose:")
			print("[ Translation:", self.pose['Translation'], ", Rotation:", self.pose['Rotation'], ']')
			
		# Display Translation and Rotation if Final pose is not None
		if self.final_pose['Pose'] is not None:
			print("Final_Pose:")
			print("[ Translation:", self.final_pose['Translation'], ", Rotation:", self.final_pose['Rotation'], ']')

		# Display for Chairs only
		if "Chair" in self.name:

			# Display Translation and Rotation if Current pose wrt Camera is not None
			if self.pose_wrt_camera['Pose'] is not None:
				print("Pose wrt Camera:")
				print("[ Translation:", self.pose_wrt_camera['Translation'], ", Rotation:", self.pose_wrt_camera['Rotation'], ']')

			#  Display Translation and Rotation if Final pose wrt Camera is not None
			if self.final_pose_wrt_camera['Pose'] is not None:
				print("Final Pose wrt Camera:")
				print("[ Translation:", self.final_pose_wrt_camera['Translation'], ", Rotation:", self.final_pose_wrt_camera['Rotation'], ']')
		print("\n")
			

# Define a Class to store Multiple Objects
class Objects:

	# Define the Init Function
	def __init__(self):
		
		# Read Objects poses from JSON file
		with open('saved_objects_poses.json', 'r') as file:
			self.objects_data = json.load(file)['objects']

		# Initialise Poses of Cameras
		self.cameras = self.initialise_poses_of_object(object_name = 'Camera')

		# Initialise Poses of Wall markers
		self.walls = self.initialise_poses_of_object(object_name = 'Wall')

		# Initialise Poses of Chairs
		self.chairs = self.initialise_poses_of_object(object_name = 'Chair')

		# Initialise Poses of Waypoints
		self.waypoints = self.initialise_poses_of_object(object_name = 'Waypoint')
	

	# Define a Function to Initialise Poses of given Object
	def initialise_poses_of_object(self, object_name):

		# Initialise List to store Poses for given Object
		objects = []

		# For every Item in the Dictionary
		for item in self.objects_data:
			
			# If Name of object matches given object_name
			if object_name in item['name']:

				# Create object
				object = Object(	
									aruco_id = item['id'],
									name = item['name'],
									translation = item['translation'],
									rotation = item['rotation']
							   )
				
				# Append Object into List
				objects.append(object)
		
		# Return Objects
		return objects
	

	# Define a Function to get the Name of the object from ArUco ID
	def get_name_of_object(self, aruco_id):

		# For every Item in the Dictionary
		for item in self.objects_data:
			
			# If ID of object matches given aruco_id
			if aruco_id == item['id']:

				# Return the Name of Object
				return item['name']
	
	
	# Define a Function to get Pose of given Object
	def get_pose_of_object(self, object_name):

		# For every Item in the Dictionary
		for item in self.objects_data:
			
			# If Name of object matches given object_name
			if object_name in item['name']:

				# Return Pose of Object
				return compute_pose_from_components(
														translation = item["translation"],
														rotation = item["rotation"]
													)


	# Define a Function to get Chairs that are Unarranged around Table
	def get_unarranged_chairs(self):

		# Initialise Unarranged chairs
		unarranged_chairs = []

  		# Initialise Arranged chairs
		arranged_chairs = []

		# For every Chair
		for object in self.chairs:

			# If Chair is not Arranged
			if not object.is_pose_at_final_pose():

				# Append chair to Unarranged chairs list
				unarranged_chairs.append(object)

   			# If Chair is Arranged
			else:

				# Append chair to Arranged chairs list
				arranged_chairs.append(object)

		# Display Details for Arranged Chairs
		print("Arranged Chairs: ")
		for chair in arranged_chairs:
			print(chair.name, ": ", chair.displacement)
		
		# Display Details for Unarranged Chairs
		print("Unarranged Chairs: ")
		for chair in unarranged_chairs:
			print(chair.name, ": ", chair.displacement)

		# Return the List of Unarranged chairs
		return unarranged_chairs		
	

	# Define a Function to Display all Objects
	def display(self):

		# Display Data for every Camera
		print("----- ArUco on Cameras -----\n")
		for object in self.cameras:
			object.display()
		
		# Display Data for every Wall
		print("----- ArUco on Walls -----\n")
		for object in self.walls:
			object.display()
		
		# Display Data for every Chair
		print("----- ArUco on Chairs -----\n")
		for object in self.chairs:
			object.display()

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
def get_pose_of_aruco_tag(aruco_tags_data_wrt_frame, object_id):

	# If there are AruCo tags detected in the Frame
	if aruco_tags_data_wrt_frame is not None:

		# For every AruCo tag detected in frame
		for aruco_tag_data_wrt_frame in aruco_tags_data_wrt_frame:

			# If AruCo ID matches given Ibject ID
			if object_id == aruco_tag_data_wrt_frame['ID']:

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


# Define a Function to Calculate Pose from Translation and Rotation components
def compute_pose_from_components(translation, rotation, degrees = True):

	# Initialise Transformation matrix
	transformation_matrix = np.zeros((4, 4))

	# Frame Transformation matrix
	transformation_matrix[:3, 3] = np.array(translation).T

	# If Rotation is in Degrees
	if degrees:
		transformation_matrix[:3, :3] = R.from_euler('xyz', rotation, degrees = True).as_matrix()
	
	# If Rotation is in Vectors
	else:
		transformation_matrix[:3, :3] = cv2.Rodrigues(np.array(rotation))[0]
	transformation_matrix[3, 3] = 1

	# Return Transformation matrix
	return round_matrix_list(transformation_matrix, 3)


# Define a Function to Get Translation and Rotation Angles from Pose for Chair only
def get_components_from_pose_for_chair(pose, degrees = True):

	# Round off Pose matrix and get Translation vector
	pose = round_matrix_list(pose, 3)
	translation = pose[:3, 3]

	# Get Rotation matrix
	rotation_matrix = pose[:3, :3]
	rotation = R.from_matrix(rotation_matrix)

	# If Rotation is in Degrees
	if degrees:
		
		# In ZYX Rotation order, calculate Euler angles
		rx = 0
		ry = round(math.degrees(math.asin(-rotation_matrix[2][0])))
		if rotation_matrix[0][1] > 0:
			rz = -19.2
		else:
			rz = 19.2
		rotation = [rx, ry , rz]
	
	# If Rotation is in Quartenion
	else:
		rotation = rotation.as_quat()

	# Approximate and Return Translation and Rotation
	translation = round_float_list(translation, 3)
	rotation = round_float_list(rotation, 3)
	return translation, rotation


# Define a Function to Get Translation and Rotation Quartenion/Angles from Pose
def get_components_from_pose(pose, degrees = True):

	# Round off Pose matrix and get Translation vector
	pose = round_matrix_list(pose, 3)
	translation = pose[:3, 3]

	# Get Rotation matrix
	rotation_matrix = pose[:3, :3]
	rotation = R.from_matrix(rotation_matrix)

	# If Rotation is in Degrees
	if degrees:
		
		# Get Euler angles in a specific sequence (e.g., 'zyx')
		# 'zyx' is a common convention, representing rotations around Z, then Y, then X.
		# You can specify the order as per your application's requirements (e.g., 'xyz', 'YXZ', etc.)
		rotation = R.from_matrix(rotation_matrix).as_euler('xyz', degrees = True)
			
	# If Rotation is in Quartenion
	else:
		rotation = rotation.as_quat()

	# Approximate and Return Translation and Rotation
	translation = round_float_list(translation, 3)
	rotation = round_float_list(rotation, 3)
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


# Define a Function to get Optimal path from Waypoint to another Waypoint
def get_waypoints_path(from_waypoint, to_waypoint):

    # Initialise Array of Waypoint Names
    waypoint_names = []
    for i in range(8):
        waypoint_names.append("Waypoint_" + str(i+1))
    
    # Return the Shortest Path around Waypoints
    i, j, n = waypoint_names.index(from_waypoint.name), waypoint_names.index(to_waypoint.name), len(waypoint_names)
    rot = waypoint_names[i:] + waypoint_names[:i]
    return rot[:(j-i)%n+1] if (j-i)%n <= (i-j)%n else [rot[0]] + rot[:-(i-j)%n-1:-1]
        