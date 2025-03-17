# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import utils


# Define a Class for Coordinate frame Transformation 
class Coordinate_Frame_Transformation:

    # Define the Initialisation Function
    def __init__(self, name, reference_frame, translation_vector, rotation_angles):
        
        # Initialise all Data members
        self.name = name
        self.translation_vector = np.array(utils.round_float_list(translation_vector, 3))
        self.rotation_angles = np.array(utils.round_float_list(rotation_angles, 3))
                                        
        # Compute the Rotation matrix from Euler angles
        self.rotation_matrix = self.get_rotation_matrix_from_angles()

        # Compute Transformation matrix
        self.transformation_matrix = self.get_transformation_matrix()

        # If no Reference frame is given
        if reference_frame is None:
            self.reference_frame = None

        # Else when Reference frame is given
        else:

            # Compute the Translation vector wrt Reference frame
            self.translation_vector = reference_frame.transformation_matrix @ np.append(self.translation_vector, 1).T
            self.translation_vector = np.array(utils.round_float_list(self.translation_vector[:3], 3))

            # Compute the Rotation matrix wrt Reference frame
            self.rotation_matrix = reference_frame.rotation_matrix @ self.rotation_matrix

            # Compute the Rotation angles wrt Reference frame
            self.rotation_angles = self.get_rotation_angles_from_matrix()

            # Compute Transformation matrix
            self.transformation_matrix = self.get_transformation_matrix()

            # Update Reference Frame
            self.reference_frame = world_frame

    # Define a Function to Compute Rotation matrix from Euler angles in degrees
    def get_rotation_matrix_from_angles(self):
        
        # Compute and Return Rotation matrix 
        rotation_matrix = R.from_euler('xyz', self.rotation_angles, degrees = True).as_matrix()
        return utils.round_matrix_list(rotation_matrix, 3)
    
    # Define a Function to Compute Rotation angles from Matrix
    def get_rotation_angles_from_matrix(self):

        # Compute and Return Rotation angles
        rotation_angles = R.from_matrix(self.rotation_matrix)
        rotation_angles = rotation_angles.as_euler('xyz', degrees = True)
        return utils.round_float_list(rotation_angles, 3)

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
        print("Reference frame: ", self.reference_frame.name)
        print("Translation vector: ", self.translation_vector)
        print("Rotation angles: ", self.rotation_angles)
        print("Rotation matrix: \n", self.rotation_matrix)
        print("Transformation matrix: \n", self.transformation_matrix)


# Create a Coordinate frame for World frame
world_frame = Coordinate_Frame_Transformation(
                                                name = 'World',
                                                reference_frame = None,
                                                translation_vector = [0, 0, 0],
                                                rotation_angles = [0, 0, 0]
                                             )

# Create a Coordinate frame for Camera frame wrt World frame
camera_frame = Coordinate_Frame_Transformation(
                                                name = 'Camera',
                                                reference_frame = world_frame,
                                                translation_vector = [0, 0, 0],
                                                rotation_angles = [-90, 0, 0]
                                              )


# Define a Function to Compute AruCo tags pose from Camera frame to World frame
def compute_aruco_tags_data_to_world_frame(aruco_tags_data_wrt_camera_frame):

    # Initialise list to store AruCo tags data wrt World frame
    aruco_tags_data_wrt_world_frame = []

    # For every AruCo tag in Camera frame
    for aruco_tag_data_wrt_camera_frame in aruco_tags_data_wrt_camera_frame:

        # Compute the AruCo tag data wrt World frame
        aruco_tag_data_wrt_world_frame = Coordinate_Frame_Transformation(
                                                                            name = aruco_tag_data_wrt_camera_frame['Name'],
                                                                            reference_frame = camera_frame,
                                                                            translation_vector = aruco_tag_data_wrt_camera_frame['Translation'],
                                                                            rotation_angles = aruco_tag_data_wrt_camera_frame['Rotation']
                                                                        )
        
        # Append AruCo tag data into List
        aruco_tags_data_wrt_world_frame.append(aruco_tag_data_wrt_world_frame)
    
    # Return AruCo tags data wrt World frame
    return aruco_tags_data_wrt_world_frame