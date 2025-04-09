# Import Necessary SPOT Scripts
from spot_robot import make_SPOT_stand
from spot_robot import DetectFiducial
import bosdyn

# Import Necessary Scripts
import read_video_stream
import pose_estimation
import coordinate_transformations
import utils
import numpy as np

# Import Necessary Libraries
import time
import cv2
import os


# Define a Function to Setup and Configure SPOT Robot
def setup_and_configure_robot():

    # Define the SPOT Robot Credentials
    os.environ['BOSDYN_CLIENT_USERNAME'] = "admin"
    os.environ['BOSDYN_CLIENT_PASSWORD'] = "pvwmr4j08osj"
    robot_ip = "192.168.80.3"

    # Setup Configurations for SPOT
    bosdyn.client.util.setup_logging(False)
    sdk = bosdyn.client.create_standard_sdk('WalkToObjectClient')
    robot = sdk.create_robot(robot_ip)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # Return the Robot object
    return robot


# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_APRILTAG_36h11

# Read Camera Calibration parameters
camera_calibration_params = utils.read_camera_calibration_params()

'''
# Configure and Stream Realsense Pipeline
main_camera_pipeline = read_video_stream.configure_and_stream_pipeline()

# Configure SPOT robot and make it Stand
robot = setup_and_configure_robot()
make_SPOT_stand(robot)

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:
        
        # Read Image frames from Pipelines
        main_camera_frame = read_video_stream.read_frames_from_pipelines(main_camera_pipeline)
        
        # Get the Pose of AruCo tags in Main Camera frame
        image_with_aruco_poses, aruco_tags_data_wrt_camera_frame = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, aruco_type, camera_calibration_params['Main_Camera'])

        print("AruCo tags wrt World frame...")
        print(aruco_tags_data_wrt_camera_frame)

        # Convert the Poses of AruCo tags from Camera frame to World frame
        #aruco_tags_data_wrt_world_frame = coordinate_transformations.compute_aruco_tags_data_to_world_frame(aruco_tags_data_wrt_camera_frame)

        # Copmute the Gripper pose to Grasp chair in World frame
        #gripper_pose_to_grasp_chair_wrt_world_frame = coordinate_transformations.compute_gripper_pose_to_grasp_chair(aruco_tags_data_wrt_world_frame)
        
        # Display the Image from Main Camera
        cv2.imshow('Main_Camera', image_with_aruco_poses)
        cv2.waitKey(1)

        # Detect the Fiducials in the Robot's environment
        aruco_tags_data_wrt_spot_body_frame = DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
        print(aruco_tags_data_wrt_spot_body_frame)

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_camera_pipeline.stop()

'''

# Read Image frames from Pipelines
main_camera_frame = cv2.imread('scene.jpg')
        
# Get the Pose of AruCo tags in Main Camera frame
image_with_aruco_poses, aruco_tags_data_wrt_camera_frame = pose_estimation.get_pose_of_aruco_tags(main_camera_frame, aruco_type, camera_calibration_params['Main_Camera'])
print(aruco_tags_data_wrt_camera_frame)
print('\n')
    
detected_fiducials = [{'Name': 'Main_Origin', 'Translation': [-4.104, -0.595, 0.369], 'Rotation': [169.941, -73.369, 14.078]}, {'Name': 'Secondary_Origin', 'Translation': [0.571, -1.466, 0.288], 'Rotation': [151.621, -88.788, 135.619]}]
print(detected_fiducials)

cv2.imshow('Main_Camera', image_with_aruco_poses)
cv2.waitKey(0)