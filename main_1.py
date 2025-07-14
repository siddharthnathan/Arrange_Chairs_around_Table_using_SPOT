# Import Necessary Scripts
import read_video_stream
import pose_estimation
import utils
import time

# Import Necessary Libraries
import cv2


############################################################################################# CONFIGURATION ################################################################################

# Define the type of AruCo marker used in Environment
aruco_type = cv2.aruco.DICT_APRILTAG_36h11

# Read Camera Calibration parameters
camera_calibration_params = utils.read_camera_calibration_params()

# Configure and Stream Realsense Pipeline for both Cameras
camera_1_pipeline, camera_2_pipeline = read_video_stream.configure_and_stream_pipeline()
time.sleep(1)

############################################################################################# INITIALISATION ###############################################################################

# Read the Image frames
camera_1_frame = read_video_stream.read_frames_from_pipelines(camera_1_pipeline)
camera_2_frame = read_video_stream.read_frames_from_pipelines(camera_2_pipeline)

# Get the Pose of AruCo tags in both Camera frames
image_1_with_aruco_tags_pose, aruco_tags_data_wrt_camera_1_frame = pose_estimation.get_pose_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1']) 
image_2_with_aruco_tags_pose, aruco_tags_data_wrt_camera_2_frame = pose_estimation.get_pose_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])

# Get the Pose of both Cameras wrt Origin
camera_1_pose, camera_2_pose = pose_estimation.get_poses_of_cameras(aruco_tags_data_wrt_camera_1_frame, aruco_tags_data_wrt_camera_2_frame)

############################################################################################### MAIN PROGRAM ###############################################################################

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:

        # Read Image frames from both Cameras
        camera_1_frame = read_video_stream.read_frames_from_pipelines(camera_1_pipeline)
        camera_2_frame = read_video_stream.read_frames_from_pipelines(camera_2_pipeline)
            
        # Get the Pose of AruCo tags in Camera 1 frame
        image_1_with_aruco_tags_pose, aruco_tags_data_wrt_camera_1_frame = pose_estimation.get_pose_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1'])        
        utils.display_image('Camera_1', image_1_with_aruco_tags_pose)
        
        # Get the Pose of AruCo tags in Camera 2 frame
        image_2_with_aruco_tags_pose, aruco_tags_data_wrt_camera_2_frame = pose_estimation.get_pose_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])        
        utils.display_image('Camera_2', image_2_with_aruco_tags_pose)

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break
    
# Stop streaming finally
finally:
    camera_1_pipeline.stop()
    camera_2_pipeline.stop()