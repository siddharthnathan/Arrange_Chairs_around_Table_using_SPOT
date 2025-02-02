# Import Necessary Scripts
import read_video_stream
import aruco_tag_detection
import pose_estimation

# Import Necessary Libraries
import numpy as np
import time
import cv2
import os


# Configure and Stream Realsense Pipeline
main_cam_pipeline, side_cam_pipeline = read_video_stream.configure_and_stream_pipeline()

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:
        
        # Read Image frames from Pipelines
        main_camera_frame, side_camera_frame = read_video_stream.read_frames_from_pipelines(main_cam_pipeline, side_cam_pipeline)
        
        '''
        if cv2.waitKey(1) == ord('s'):
            cv2.imwrite('Camera_Calibration/checkerboard_images/' + str(img_count) + '.jpg', main_camera_frame)
            img_count = img_count + 1
        
        aruco_tags_detected_main_cam = aruco_tag_detection.detect_aruco_markers_in_frame(main_camera_frame, 'Main_Camera', aruco_type = cv2.aruco.DICT_APRILTAG_36h11)
        print(aruco_tags_detected_main_cam)

        image_with_aruco_tags_pose, poses_of_aruco_tags = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, cv2.aruco.DICT_APRILTAG_36h11, np.load('Camera_Calibration/calibration_matrix.npy'), np.load('Camera_Calibration/distortion_coefficients.npy'))
        print(poses_of_aruco_tags)
        cv2.imshow('Main_Camera', image_with_aruco_tags_pose)
        cv2.waitKey(1)
        '''

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_cam_pipeline.stop()
    side_cam_pipeline.stop()