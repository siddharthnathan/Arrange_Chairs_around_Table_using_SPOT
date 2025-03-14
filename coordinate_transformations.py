# Import Necessary Libraries
import numpy as np
import utils


# Define a Class for Coordinate Frame
class Coordinate_frame:

    # Define an Init function
    def __init__(self, name, translation_vector, rotation_angles):

        # Set values for Data members
        self.name = name
        self.translation_vector = np.array(translation_vector).T
        self.rotation_angles = np.array(rotation_angles)

        # Get the Rotation matrix and Transformation matrix
        self.rotation_matrix = utils.get_rotation_matrix_from_angles(self.rotation_angles)

        # Get the Transformation matrix
        self.transformation_matrix = utils.get_transformation_matrix(self.translation_vector, self.rotation_matrix)

      
# Define a Class for Coordinate Frame Transformation
class Coordinate_frame_Transformation:

    # Define an Init function
    def __init__(self, ref_coordinate_frame, aruco_tag_data):

        # Set values for Data members
        self.name = aruco_tag_data.name
        self.reference_frame = ref_coordinate_frame.name
        self.coords_wrt_ref_frame = np.append(aruco_tag_data.translation_vector, 1).T
        self.angles_wrt_ref_frame = np.append(aruco_tag_data.rotation_angles, 1).T

        # Compute the Translation vector wrt Reference frame
        self.translation_vector = (ref_coordinate_frame.transformation_matrix @ self.coords_wrt_ref_frame)[:3]
        
        # Compute the Rotation angles wrt Reference frame
        self.rotation_angles = (ref_coordinate_frame.transformation_matrix @ self.angles_wrt_ref_frame)[:3] + ref_coordinate_frame.rotation_angles

        # Create Coordinate frame object
        self.coordinate_frame = Coordinate_frame(self.name, self.translation_vector, self.rotation_angles) 
    
    # Define a Function to Display Data about Coordinate frame
    def display_data(self):

        # Print every Information to Display about Coordinate frame
        print("Name: ", self.name)
        print("Reference frame: ", self.reference_frame)
        print("Translation vector: ", self.coordinate_frame.translation_vector)
        print("Rotation angles: ", self.coordinate_frame.rotation_angles)
        print("Transformation matrix: ", self.coordinate_frame.transformation_matrix)

    

# Define the Camera frame wrt World frame
camera_frame_wrt_world_frame = Coordinate_frame(
                                                    name = 'Camera_wrt_World', 
                                                    translation_vector = [0, 0, 0], 
                                                    rotation_angles = [-90, 0, 0]
                                               )


# Define a Function to determine Poses from Camera frame to World frame
def get_pose_from_camera_frame_to_world_frame(aruco_tags_data):

    # Initialise List to store AruCo tags data wrt World frame
    aruco_tags_data_wrt_world_frame = []

    # For every AruCo tag Pose
    for i in range(len(aruco_tags_data)):

        # Compute Pose of AruCo tag wrt World frame from Camera frame
        aruco_tag_data_wrt_world_frame = Coordinate_frame_Transformation(camera_frame_wrt_world_frame, aruco_tags_data[i])

        # Append the AruCo tag data into List
        aruco_tags_data_wrt_world_frame.append(aruco_tag_data_wrt_world_frame)
    
    # Return the List of AruCo tags Data wrt World frame
    return aruco_tags_data_wrt_world_frame
