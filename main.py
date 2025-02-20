# Import Necessary Scripts
import read_video_stream
import aruco_tag_detection
import pose_estimation
import utils

# Import Necessary Libraries
import numpy as np
import time
import cv2
import os


# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_APRILTAG_36h11

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
        
        # Detect AruCo tags in Main Camera frame
        aruco_tags_detected_main_cam = aruco_tag_detection.detect_aruco_markers_in_frame(main_camera_frame, 'Main_Camera', aruco_type)

        # Get the Pose of AruCo tags in Main Camera frame
        image_with_aruco_tags_pose, poses_of_aruco_tags = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, aruco_type, camera_calibration_params)
        
        cv2.imshow('Main_Camera', image_with_aruco_tags_pose)
        cv2.waitKey(1)
        

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_camera_pipeline.stop()
    side_camera_pipeline.stop()