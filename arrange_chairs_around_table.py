# Import Necessary Scripts
import spot_robot_commands
import read_video_stream
import pose_estimation
import utils
import time

# Import Necessary Libraries
import cv2


# Define a Function to Set Poses of Objects at Initial and Final Configuration
def set_poses_of_objects_at_start_and_final_states(aruco_type, camera_calibration_params):

    # Compute the Poses of both Cameras using the Initial Images from both cameras
    initial_images = [cv2.imread('initial_image_1.jpg'), cv2.imread('initial_image_2.jpg')]
    poses_of_cameras = pose_estimation.get_poses_of_cameras(initial_images, aruco_type, camera_calibration_params)

    # Get the Object mapping with their AruCo IDs
    objects_with_aruco_ids = utils.read_mapping_of_objects()

    # Initialise Objects in Scene with their AruCo ID, Name, Poses
    objects = utils.Objects(objects_with_aruco_ids)

    # Update the Poses of Origins AruCo markers using Initial Images from both cameras
    objects = pose_estimation.update_poses_of_origins(initial_images, objects, poses_of_cameras, aruco_type, camera_calibration_params)

    # Update the Goal Configuration Poses of Chairs using Final Images from both cameras
    final_images = [cv2.imread('final_image_1.jpg'), cv2.imread('final_image_2.jpg')]
    objects = pose_estimation.update_final_poses_of_chairs(final_images, objects, poses_of_cameras, aruco_type, camera_calibration_params)

    # Return the Poses of Cameras and Objects
    return poses_of_cameras, objects


# Define the Main Function
def main():

    # Define the type of AruCo marker used in Environment
    aruco_type = cv2.aruco.DICT_APRILTAG_36h11

    # Read Camera Calibration parameters for both Cameras
    camera_calibration_params = read_video_stream.read_camera_calibration_params()

    # Configure and Stream Realsense Camera Pipelines for both Cameras
    camera_1_pipeline, camera_2_pipeline = read_video_stream.configure_and_stream_pipeline()

    # Set Poses of Objects at Initial and Final Configuration
    poses_of_cameras, objects = set_poses_of_objects_at_start_and_final_states(aruco_type, camera_calibration_params)

    # Try block
    try:

        # Read Image frames continuously
        while True:

            # Read Image frames from both Cameras
            camera_1_frame = read_video_stream.read_frame_from_pipeline(camera_1_pipeline)
            camera_2_frame = read_video_stream.read_frame_from_pipeline(camera_2_pipeline)
            
            # Get the Pose of AruCo tags wrt SPOT Body Frame
            aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial.detect_aruco_tags_wrt_spot_body_frame()

            # Update the Poses of AruCo markers using Images from both cameras
            objects = pose_estimation.update_poses_of_objects(
                                                                [camera_1_frame, camera_2_frame], aruco_tags_data_wrt_spot_frame, 
                                                                objects, poses_of_cameras, aruco_type, camera_calibration_params
                                                             )

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