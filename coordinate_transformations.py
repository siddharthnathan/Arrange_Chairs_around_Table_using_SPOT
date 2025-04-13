# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import utils


# Define a Class to create Coordinate Frame object
class Coordinate_Frame:

    # Define the Initialisation function
    def __init__(self, aruco_tag_dict, reference_frame):
        
        # Initialise all Data members
        self.name = aruco_tag_dict['Name']
        self.reference_frame = reference_frame
        self.translation_vector = np.array(aruco_tag_dict['Translation'])
        self.rotation_angles = np.array(aruco_tag_dict['Rotation'])

        # Compute the Rotation matrix from Euler angles
        self.rotation_matrix = self.get_rotation_matrix_from_angles()

        # Compute Transformation matrix
        self.transformation_matrix = self.get_transformation_matrix()

    # Define a Function to Compute Rotation matrix from Euler angles in degrees
    def get_rotation_matrix_from_angles(self):
        
        # Compute and Return Rotation matrix 
        rotation_matrix = R.from_euler('zyx', self.rotation_angles, degrees = True).as_matrix()
        return utils.round_matrix_list(rotation_matrix, 3)

    # Define a Function to get Transformation matrix
    def get_transformation_matrix(self):

        # Frame Translation vector
        translation_vector = np.append(self.translation_vector, 1).T

        # Initialise Transformation matrix
        self.transformation_matrix = np.zeros((4, 4))

        # Frame and Return Transformation matrix
        self.transformation_matrix[:4, 3] = translation_vector
        self.transformation_matrix[:3, :3] = self.rotation_matrix
        return self.transformation_matrix

    # Define a Function to Display Data members
    def display(self):

        # Display the Necessary Information
        print("Name: ", self.name)
        print("Reference frame: ", self.reference_frame)
        print("Translation vector: ", self.translation_vector)
        print("Rotation angles: ", self.rotation_angles)
        print("\n")


# Define a Function to Create Coordinate frames objects
def create_coordinate_frames(aruco_tags_data, reference_frame):

    # Initialise AruCo tags object list
    aruco_tags_objects = []

    # For every AruCo tag data
    for aruco_tag_data in aruco_tags_data:

        # Create a Coordinate frame object
        aruco_tag_object = Coordinate_Frame(aruco_tag_data, reference_frame)

        # Append AruCo tag object into Object list
        aruco_tags_objects.append(aruco_tag_object)
    
    # Return the AruCo tags object list
    return aruco_tags_objects


# Define a Class for Coordinate frame Transformation wrt Reference frame
class Coordinate_Frame_Transformation:

    # Define the Initialisation Function
    def __init__(self, frame, reference_frame):
        
        # Initialise all Data members
        self.frame = frame
        self.reference_frame = reference_frame

        # Compute the Transformation matrix
        self.transformation_matrix = utils.round_matrix_list(np.linalg.inv(self.reference_frame.transformation_matrix) @ self.frame.transformation_matrix, 3)
    
        # Extract the Translation vector
        self.translation_vector = self.transformation_matrix[:3, 3]
        self.translation_vector = [self.translation_vector[0], self.translation_vector[1], self.translation_vector[2]]

        # Extract the Rotation matrix
        self.rotation_matrix = self.transformation_matrix[:3, :3]

        # Compute the Rotation angles from Rotation matrix
        self.rotation_angles = self.get_rotation_angles_from_matrix()
       
    # Define a Function to Compute Rotation angles from Matrix
    def get_rotation_angles_from_matrix(self):

        # Compute and Return Rotation angles
        rotation_angles = R.from_matrix(self.rotation_matrix)
        rotation_angles = rotation_angles.as_euler('zyx', degrees = True)
        return utils.round_float_list(rotation_angles, 3)[::-1]

    # Define a Function to Display Data members
    def display(self):
        print("Name: ", self.frame.name)
        print("Reference frame: ", self.reference_frame.name)
        print("Translation vector: ", self.translation_vector)
        print("Rotation angles: ", self.rotation_angles)
        print("\n")


# Define a Function to Compute Chair AruCo pose wrt SPOT Body frame
def compute_chair_pose_wrt_spot(aruco_tags_data_wrt_camera_frame, aruco_tags_data_wrt_spot_frame):

    # Get the Poses of Main origin and Secondary Origin wrt SPOT frame
    main_origin_pose_wrt_spot_frame = utils.get_aruco_tag_data(aruco_tags_data_wrt_spot_frame, 'Main_Origin')
    secondary_origin_pose_wrt_spot_frame = utils.get_aruco_tag_data(aruco_tags_data_wrt_spot_frame, 'Secondary_Origin')

    # Compute the Pose of Chair AruCo wrt Camera frame
    chair_aruco_pose_wrt_camera_frame = utils.get_aruco_tag_data(aruco_tags_data_wrt_camera_frame, 'Chair')

    # If Main Origin is detected by SPOT
    if main_origin_pose_wrt_spot_frame is not None:

        # Determine the Main Origin pose wrt Camera frame
        main_origin_pose_wrt_camera_frame = utils.get_aruco_tag_data(aruco_tags_data_wrt_camera_frame, 'Main_Origin')

        # Compute the Pose of SPOT body wrt Camera frame
        spot_body_pose_wrt_camera_frame = main_origin_pose_wrt_camera_frame.transformation_matrix @ np.linalg.inv(main_origin_pose_wrt_spot_frame.transformation_matrix)
    
    # Else if Secondary Origin is detected by SPOT
    elif secondary_origin_pose_wrt_spot_frame is not None:

        # Determine the Secondary Origin pose wrt Camera frame
        secondary_origin_pose_wrt_camera_frame = utils.get_aruco_tag_data(aruco_tags_data_wrt_camera_frame, 'Secondary_Origin')

        # Compute the Pose of SPOT body wrt Camera frame
        spot_body_pose_wrt_camera_frame = secondary_origin_pose_wrt_camera_frame.transformation_matrix @ np.linalg.inv(secondary_origin_pose_wrt_spot_frame.transformation_matrix)
        
    # Compute the Chair AruCo pose wrt SPOT Body frame
    chair_aruco_pose_wrt_spot_frame = spot_body_pose_wrt_camera_frame @ chair_aruco_pose_wrt_camera_frame.transformation_matrix

    # Return the Chair AruCo pose wrt SPOT Body frame
    return chair_aruco_pose_wrt_spot_frame