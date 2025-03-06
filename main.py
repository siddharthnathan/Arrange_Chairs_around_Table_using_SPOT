# Import Necessary Scripts
import read_video_stream
import pose_estimation
import coordinate_transformations
import utils

# Import Necessary Libraries
import numpy as np
import time
import cv2
import os


# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_4X4_50

# Read Camera Calibration parameters
camera_calibration_params = utils.read_camera_calibration_params()

# Configure and Stream Realsense Pipeline
main_camera_pipeline, side_camera_pipeline = read_video_stream.configure_and_stream_pipeline()

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:
        
        # Read Image frames from Pipelines
        main_camera_frame, side_camera_frame = read_video_stream.read_frames_from_pipelines(main_camera_pipeline, side_camera_pipeline)
        
        # Get the Pose of AruCo tags in Main Camera frame
        image_with_aruco_poses, poses_of_aruco_tags = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, aruco_type, camera_calibration_params['Main_Camera'])
        
        # Convert the Poses of AruCo tags from Camera frame to World frame
        poses_of_aruco_tags = coordinate_transformations.get_pose_from_camera_frame_to_world_frame(poses_of_aruco_tags)

        # Display the Image from Main Camera
        cv2.imshow('Main_Camera', image_with_aruco_poses)
        cv2.waitKey(1)

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:

    main_camera_pipeline.stop()
    side_camera_pipeline.stop()