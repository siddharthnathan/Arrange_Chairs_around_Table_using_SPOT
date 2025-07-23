# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2


# Define a Class to store the AruCo ID, Name, Pose of Object
class Object:

	# Define the Init Function
	def __init__(self, aruco_id):

		# Define the Mapping of Object names with AruCo IDs
		self.objects_with_aruco_ids = {
											1: 'Left_Wall_1',
											2: 'Left_Wall_2',
											3: 'Origin',
											4: 'Right_Wall_1',
											5: 'Right_Wall_2',
											6: 'Chair_1',
											7: 'Chair_2',
											8: 'Chair_3',
											9: 'Chair_4',
									 }
		
		# Define the Goal Configuration Poses of Chairs
		self.goal_configuration_poses_of_chairs = {
														'Chair_1': np.array([
																				[  1,  0,  0,  0.2 ],
																				[  0,  1,  0,  0.4 ],
																				[  0,  0,  1,  0.1 ],
																				[  0,  0,  0,  1   ]
																			]),
														'Chair_2': np.array([
																				[  1,  0,  0,  0.5 ],
																				[  0,  1,  0,  0.3 ],
																				[  0,  0,  1,  0.8 ],
																				[  0,  0,  0,  1   ]
																			]),
														'Chair_3': np.array([
																				[  1,  0,  0,  0.6 ],
																				[  0,  1,  0,  0.2 ],
																				[  0,  0,  1,  0.9 ],
																				[  0,  0,  0,  1   ]
																			]),
														'Chair_4': np.array([
																				[  1,  0,  0,  0.9 ],
																				[  0,  1,  0,  0.4 ],
																				[  0,  0,  1,  0.5 ],
																				[  0,  0,  0,  1   ]
																			]),
												  }
		
		# Initialise the AruCo ID, Name, Pose, Final Pose of Object
		self.aruco_id = aruco_id
		self.name = self.objects_with_aruco_ids[self.aruco_id]

		# If Object name is Origin
		if "Origin" in self.name:
			self.pose = np.identity(4)
		
		# Else if Object name is not Origin
		else:
			self.pose = None

		# If Object name is a Chair
		if "Chair" in self.name:
			self.final_pose = self.goal_configuration_poses_of_chairs[self.name]
		
		# Else if Object is not a Chair
		else:
			self.final_pose = None
			

	# Define a Function to Display Class members
	def display(self):

		# Display Class Members
		print("AruCo ID: ", self.aruco_id)
		print("Name: ", self.name)
		print("Pose: ", self.pose)
		print("Final_Pose: \n", self.final_pose)
		print("\n")
			

# Define a Class to store Multiple Objects
class Objects:

	# Define the Init Function
	def __init__(self, num_of_objects):
		
		# Initialise Objects in Scene
		self.objects = []
		for i in range(num_of_objects):
			self.objects.append(Object(aruco_id = i + 1))
	
	# Define a Function to get the Value of a given field from Objects list
	def get_key_value_of_object(self, aruco_id, field):

		# For every Object in List
		for object in self.objects:

			# If AruCo ID matches
			if object.aruco_id == aruco_id:

				# If field is Name, Return Name
				if field == "Name": return object.name

				# If field is Pose, Return Pose
				elif field == "Pose": return object.pose

				# If field is Final Pose, Return Final Pose
				elif field == "Final_Pose": return object.final_pose

				# Else Return None
				else: return None


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

	# For every AruCo tag wrt that Frame
	for aruco_tag_data_wrt_frame in aruco_tags_data_wrt_frame:

		# If Name matches
		if aruco_tag_data_wrt_frame['Name'] == object_name:

			# Return the Pose of AruCo tag
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