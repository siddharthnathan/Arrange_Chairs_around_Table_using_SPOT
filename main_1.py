# Import Necessary Scripts
import read_video_stream
import pose_estimation
import utils
import time

# Import Necessary Libraries
import cv2


############################################################################################# INITIALISATION ################################################################################

# Initialise the Objects with their respective AruCo IDs and Poses
objects_with_aruco_ids_and_poses = utils.initialise_objects_in_environment()

# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_APRILTAG_36h11

# Read Camera Calibration parameters for both Cameras
camera_calibration_params = read_video_stream.read_camera_calibration_params()

# Configure and Stream Realsense Camera Pipelines for both Cameras
camera_1_pipeline, camera_2_pipeline = read_video_stream.configure_and_stream_pipeline()

# Read the Image frames
camera_1_frame = read_video_stream.read_frame_from_pipeline(camera_1_pipeline)
camera_2_frame = read_video_stream.read_frame_from_pipeline(camera_2_pipeline)

# Get the Pose of AruCo tags wrt both Cameras
_, aruco_tags_data_wrt_camera_1_frame = pose_estimation.get_poses_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1']) 
_, aruco_tags_data_wrt_camera_2_frame = pose_estimation.get_poses_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])

# Calculate the Pose of both Cameras wrt Origin AruCo marker
poses_of_cameras = pose_estimation.get_poses_of_cameras(aruco_tags_data_wrt_camera_1_frame, aruco_tags_data_wrt_camera_2_frame)

############################################################################################### MAIN PROGRAM ###############################################################################

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:

        # Read Image frames from both Cameras
        camera_1_frame = read_video_stream.read_frames_from_pipelines(camera_1_pipeline)
        camera_2_frame = read_video_stream.read_frames_from_pipelines(camera_2_pipeline)
            
        # Get the Pose of AruCo tags in Camera 1 frame
        image_1_with_poses_of_aruco_tags, aruco_tags_data_wrt_camera_1_frame = pose_estimation.get_poses_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1'])        
        utils.display_image('Camera_1', image_1_with_poses_of_aruco_tags)
        
        # Get the Pose of AruCo tags in Camera 2 frame
        image_2_with_poses_of_aruco_tags, aruco_tags_data_wrt_camera_2_frame = pose_estimation.get_poses_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])        
        utils.display_image('Camera_2', image_2_with_poses_of_aruco_tags)

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break
    
# Stop streaming finally
finally:
    camera_1_pipeline.stop()
    camera_2_pipeline.stop()