# Import Necessary Libraries
import numpy as np
import utils


# Define the Rotation matrix Transformation from Camera frame to World frame
orientation_angles = np.array([-90, 0, 0])
R_from_camera_to_world_frame = utils.get_rotation_matrix_from_angles(orientation_angles)


# Define a Function to determine Poses from Camera frame to World frame
def get_pose_from_camera_frame_to_world_frame(poses_of_aruco_tags):

    # For every AruCo tag Pose
    for i in range(len(poses_of_aruco_tags)):

        # Update AruCo tag Pose with respect to World frame
        poses_of_aruco_tags[i]['Translation'] = R_from_camera_to_world_frame @ poses_of_aruco_tags[i]['Translation']
        poses_of_aruco_tags[i]['Rotation'] = R_from_camera_to_world_frame @ poses_of_aruco_tags[i]['Rotation']
        poses_of_aruco_tags[i]['Rotation'] += orientation_angles
    
    # Return the Poses of AruCo tags with respect to World frame
    return poses_of_aruco_tags
