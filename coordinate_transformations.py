# Import Necessary Libraries
import numpy as np
import utils


# Define a Function to Compute Pose of AruCo on chair wrt SPOT frame
def compute_chair_pose_wrt_spot(aruco_tags_data_wrt_camera_frame, aruco_tags_data_wrt_spot_frame):

    # Get the Pose of AruCo on chair wrt Camera frame
    chair_aruco_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Chair')

    # Get the Pose of Main origin and Secondary origin wrt SPOT frame
    main_origin_pose_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 'Main_Origin')
    secondary_origin_pose_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 'Secondary_Origin')

    # Get the Pose of Main origin and Secondary origin wrt Camera frame
    main_origin_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Main_Origin')
    secondary_origin_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Secondary_Origin')

    # If Main Origin is Detected by SPOT frame
    if main_origin_pose_wrt_spot_frame is not None:

        # Get the Pose of Main origin wrt Camera frame
        main_origin_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Main_Origin')

        # Compute the Pose of SPOT wrt Camera using Main origin
        pose_of_spot_wrt_camera = main_origin_pose_wrt_camera_frame @ np.linalg.inv(main_origin_pose_wrt_spot_frame)
    
    # Else If Secondary Origin is Detected by SPOT frame
    elif secondary_origin_pose_wrt_spot_frame is not None:
    
        # Get the Pose of Secondary origin wrt Camera frame
        secondary_origin_pose_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Secondary_Origin')

        # Compute the Pose of SPOT wrt Camera using Secondary origin
        pose_of_spot_wrt_camera = secondary_origin_pose_wrt_camera_frame @ np.linalg.inv(secondary_origin_pose_wrt_spot_frame)
    
    # Compute the Pose of AruCo on chair wrt SPOT body frame
    chair_aruco_pose_wrt_spot_frame = np.linalg.inv(pose_of_spot_wrt_camera) @ chair_aruco_pose_wrt_camera_frame

    # Return the Pose of AruCo on chair wrt SPOT frame
    return chair_aruco_pose_wrt_spot_frame