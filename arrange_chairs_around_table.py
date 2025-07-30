# Import Necessary Scripts
import spot_robot_commands
import read_video_stream
import pose_estimation
import utils
import time

# Import Necessary Libraries
import cv2


# Define the Main Function
def main():

    ############################################################################################# INITIALISATION ################################################################################

    # Get the Object mapping with their AruCo IDs
    objects_with_aruco_ids = utils.read_mapping_of_objects()

    # Define the type of AruCo marker used in Environment
    aruco_type = cv2.aruco.DICT_APRILTAG_36h11

    # Read Camera Calibration parameters for both Cameras
    camera_calibration_params = read_video_stream.read_camera_calibration_params()

    # Configure and Stream Realsense Camera Pipelines for both Cameras
    camera_1_pipeline, camera_2_pipeline = read_video_stream.configure_and_stream_pipeline()

    # Initialise Objects in Scene with their AruCo ID, Name, Poses
    objects = utils.Objects(objects_with_aruco_ids)

    # Compute the Poses of both Cameras using the Initial Images from both cameras
    poses_of_cameras = pose_estimation.get_poses_of_cameras(['initial_image_1.jpg', 'initial_image_2.jpg'], aruco_type, camera_calibration_params)

    ############################################################################################### MAIN PROGRAM ###############################################################################

    # Try block
    try:

        # Read Image frames continuously
        while True:

            # Read Image frames from both Cameras
            camera_1_frame = read_video_stream.read_frame_from_pipeline(camera_1_pipeline)
            camera_2_frame = read_video_stream.read_frame_from_pipeline(camera_2_pipeline)
                
            # Get the Pose of AruCo tags from both Camera frames
            aruco_tags_data_wrt_camera_1_frame = pose_estimation.estimate_poses_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1'])        
            aruco_tags_data_wrt_camera_2_frame = pose_estimation.estimate_poses_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])        

            # Get the Pose of AruCo tags wrt SPOT Body Frame
            aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial.detect_aruco_tags_wrt_spot_body_frame(objects)

            # Quit when Q key is Pressed
            if cv2.waitKey(1) == ord('q'):
                break
        
    # Stop streaming finally
    finally:
        camera_1_pipeline.stop()
        camera_2_pipeline.stop()


# Invoke the Main Function
if __name__ == "__main__":
    main()