# Import Necessary Libraries
import numpy as np
import utils
import math


# Define the Pose of Grasp location (Seatrest from above: X - down, Y - left) wrt AruCo on Chair (Seatrest behind chair: X - down, Y - right)
pose_of_grasp_location_wrt_aruco_on_chair = np.array([
                                                        [ 1,  0,  0, -0.3400],
                                                        [ 0, -1,  0, -0.1000],
                                                        [ 0,  0, -1, -0.0350],
                                                        [ 0,  0,  0,  1.0000]
                                                    ])


# Define a Function to Account for the Offset in Grasp pose wrt SPOT
def account_for_offset(grasp_pose_wrt_spot, offset):

    # Get the Translation and Rotation of Grasp pose wrt SPOT
    translation, rotation = utils.get_translation_and_rotation_from_pose(grasp_pose_wrt_spot, angle = True)
    
    # If Offset is due to Camera
    if offset == 'SPOT':

        # Update Translation according to the Offsets in m
        translation[0] -= 0.08
        translation[1] -= 0.10
        translation[2] += 0.01

        # Update Rotation accordingly
        rotation[0] = 0
        rotation[1] = 90
        rotation[2] = 0
    
    # If Offset is due to SPOT
    elif offset == 'Camera':

        # Update Translation according to the Offsets in m
        translation[0] += 0
        translation[1] += 0
        translation[2] += 0

        # Update Rotation accordingly
        rotation[0] = 0
        rotation[1] = 90
        rotation[2] = 0

    # Update Pose accordingly and Return
    grasp_pose_wrt_spot = utils.compute_pose_from_vectors_or_angles(translation, rotation, angle = True)
    return grasp_pose_wrt_spot


# Define a Function to Compute Grasp Pose of Chair wrt SPOT body frame
def compute_grasp_pose_of_chair(aruco_tags_data_wrt_camera_frame, aruco_tags_data_wrt_spot_frame):

    # Get the Pose of AruCo on chair wrt Camera frame
    chair_aruco_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Chair')

    # Get the Pose of Main origin and Secondary origin wrt SPOT frame
    main_origin_pose_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 'Main_Origin')
    secondary_origin_pose_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 'Secondary_Origin')

    # Get the Pose of Main origin and Secondary origin wrt Camera frame
    main_origin_pose_wrt_camera_frame = np.array([
                                                    [ 0.006, -0.624, -0.782,  2.559],
                                                    [-0.957,  0.223, -0.185,  1.132],
                                                    [ 0.29,   0.749, -0.596,  3.884],
                                                    [ 0,      0,      0,     1     ]
                                                 ])
    secondary_origin_pose_wrt_camera_frame = np.array([
                                                            [-0.115, -0.785,  0.608, -2.44 ],
                                                            [-0.909, -0.163, -0.383,  1.044],
                                                            [ 0.4,   -0.597, -0.695,  3.307],
                                                            [ 0,      0,      0,     1     ]
                                                      ])
    
    # If Main Origin is Detected by SPOT frame
    if main_origin_pose_wrt_spot_frame is not None:

        # Compute the Pose of SPOT wrt Camera using Main origin
        pose_of_spot_wrt_camera = main_origin_pose_wrt_camera_frame @ np.linalg.inv(main_origin_pose_wrt_spot_frame)
    
    # Else If Secondary Origin is Detected by SPOT frame
    elif secondary_origin_pose_wrt_spot_frame is not None:
    
        # Compute the Pose of SPOT wrt Camera using Secondary origin
        pose_of_spot_wrt_camera = secondary_origin_pose_wrt_camera_frame @ np.linalg.inv(secondary_origin_pose_wrt_spot_frame)
        
    # Compute the Pose of AruCo on chair wrt SPOT body frame
    chair_aruco_pose_wrt_spot_frame = np.linalg.inv(pose_of_spot_wrt_camera) @ chair_aruco_pose_wrt_camera_frame
    return chair_aruco_pose_wrt_spot_frame


# Define a Function to Compute the Final grasp pose of Chair wrt SPOT grav aligned body frame
def compute_final_grasp_pose_of_chair(chair_aruco_pose_wrt_spot_frame, offset):

    # Compute the Grasp pose of Chair wrt SPOT
    grasp_pose_wrt_spot = chair_aruco_pose_wrt_spot_frame @ pose_of_grasp_location_wrt_aruco_on_chair   

    # Account for Offset
    grasp_pose_wrt_spot = account_for_offset(grasp_pose_wrt_spot, offset)

    # Return the Grasp Pose of Chair wrt SPOT frame
    return grasp_pose_wrt_spot