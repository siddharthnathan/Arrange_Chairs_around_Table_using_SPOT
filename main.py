# Import Necessary SPOT Scripts
from spot_robot import setup_and_configure_robot, make_SPOT_stand, move_arm_to_location
from spot_robot import DetectFiducial

# Import Necessary Scripts
import read_video_stream
import pose_estimation
import coordinate_transformations
import utils

# Import Necessary Libraries
import time
import cv2


# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_APRILTAG_36h11

# Read Camera Calibration parameters
camera_calibration_params = utils.read_camera_calibration_params()

# Configure and Stream Realsense Pipeline
main_camera_pipeline = read_video_stream.configure_and_stream_pipeline()

# Configure SPOT robot and make it Stand
robot = setup_and_configure_robot()
make_SPOT_stand(robot)

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:
        
        # Detect the Fiducials in the Robot's environment
        aruco_tags_data_wrt_spot_body_frame = DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
        
        # Read Image frames from Pipelines
        main_camera_frame = read_video_stream.read_frames_from_pipelines(main_camera_pipeline)
        
        # Get the Pose of AruCo tags in Main Camera frame
        image_with_aruco_poses, aruco_tags_data_wrt_camera_frame = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, aruco_type, camera_calibration_params['Main_Camera'])
        
        # Get the Poses of Chair wrt Camera and SPOT frames
        pose_of_aruco_on_chair_wrt_camera_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_frame, 'Chair')
        pose_of_aruco_on_chair_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_body_frame, 'Chair')

        # If Chair is Detected in Camera and SPOT frames
        if pose_of_aruco_on_chair_wrt_camera_frame is not None or pose_of_aruco_on_chair_wrt_spot_frame is not None:

            # If Chair is not detected by SPOT
            if pose_of_aruco_on_chair_wrt_spot_frame is None:

                # Compute the Grasp Pose of Chair wrt SPOT body frame
                grasp_pose_wrt_spot = coordinate_transformations.compute_grasp_pose_of_chair(robot, aruco_tags_data_wrt_camera_frame, aruco_tags_data_wrt_spot_body_frame)

                # Move Arm to Grasp chair
                #move_arm_to_location(robot, grasp_pose_wrt_spot)
            
        # If Chair is not Detected in Camera and SPOT frames
        else:
            print("Chair is Not detected in Camera & SPOT frames")
        
        # Display the Image from Main Camera
        cv2.imshow('Main_Camera', image_with_aruco_poses)
        cv2.waitKey(1)
        time.sleep(5)
        
        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_camera_pipeline.stop()
