# Import Necessary Scripts
import read_video_stream
import pose_estimation

# Import Necessary Libraries
import utils
import cv2


# Define the Main Function
def main():

    # Define the type of AruCo marker used in Environment
    aruco_type = cv2.aruco.DICT_APRILTAG_36h11

    # Read Camera Calibration parameters for both Cameras
    camera_calibration_params = read_video_stream.read_camera_calibration_params()

    # Configure and Stream Realsense Camera Pipelines for both Cameras
    camera_1_pipeline, camera_2_pipeline = read_video_stream.configure_and_stream_pipeline()

    # Try block
    try:

        # Read Image frames continuously
        while True:

            # Read Image frames from both Cameras
            camera_1_frame = read_video_stream.read_frame_from_pipeline(camera_1_pipeline)
            camera_2_frame = read_video_stream.read_frame_from_pipeline(camera_2_pipeline)
            save_image_1, save_image_2 = camera_1_frame.copy(), camera_2_frame.copy()

            # Get the Pose of AruCo tags wrt both Cameras
            aruco_tags_data_wrt_camera_1_frame = pose_estimation.estimate_poses_of_aruco_tags(camera_1_frame, aruco_type, camera_calibration_params['Camera_1']) 
            aruco_tags_data_wrt_camera_2_frame = pose_estimation.estimate_poses_of_aruco_tags(camera_2_frame, aruco_type, camera_calibration_params['Camera_2'])

            # Save Images of Origin AruCo markers when required Markers are Detected
            if utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Origin') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Right_Wall_1') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Right_Wall_2') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Origin') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Left_Wall_1') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Left_Wall_2') is not None:
                print("Saved AruCo marker Images for Origins")
                cv2.imwrite('aruco_markers_for_origins_image_1.jpg', save_image_1)
                cv2.imwrite('aruco_markers_for_origins_image_2.jpg', save_image_2)
            
            # # Save Images of Origin AruCo markers when required Markers are Detected
            elif utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Chair_1') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Chair_2') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Chair_3') is not None and utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Chair_4') is not None:
                print("Saved AruCo marker Images for Chairs")
                cv2.imwrite('aruco_markers_for_chairs_image_1.jpg', save_image_1)
                cv2.imwrite('aruco_markers_for_chairs_image_2.jpg', save_image_2)
            
            # Quit when Q key is Pressed
            elif cv2.waitKey(1) == ord('q'):
                break

            # Else Continue
            else:
                continue
        
    # Stop streaming finally
    finally:
        camera_1_pipeline.stop()
        camera_2_pipeline.stop()


# Invoke the Main Function
if __name__ == "__main__":
    main()