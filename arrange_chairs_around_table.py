# Import Necessary Scripts
import spot_robot_commands
import read_video_stream
import pose_estimation
import utils
import time

# Import Necessary Libraries
import numpy as np
import cv2


# Define a Function to Set Poses of Objects at Initial and Final Configuration
def set_poses_of_objects_at_start_and_final_states(aruco_type, camera_calibration_params):

    # Compute the Poses of both Cameras using the Initial Images from both cameras
    initial_images = [cv2.imread('aruco_markers_for_origins_image_1.jpg'), cv2.imread('aruco_markers_for_origins_image_2.jpg')]
    poses_of_cameras = pose_estimation.get_poses_of_cameras(initial_images, aruco_type, camera_calibration_params)
    
    # Get the Object mapping with their AruCo IDs
    objects_with_aruco_ids = utils.read_mapping_of_objects()

    # Initialise Objects in Scene with their AruCo ID, Name, Poses
    objects = utils.Objects(objects_with_aruco_ids)
    
    # Update the Poses of Origins AruCo markers using Initial Images from both cameras
    initial_images = [cv2.imread('aruco_markers_for_origins_image_1.jpg'), cv2.imread('aruco_markers_for_origins_image_2.jpg')]
    objects = pose_estimation.update_poses_of_origins(initial_images, objects, poses_of_cameras, aruco_type, camera_calibration_params)
    
    # Update the Goal Configuration Poses of Chairs using Final Images from both cameras
    final_images = [cv2.imread('aruco_markers_for_chairs_image_1.jpg'), cv2.imread('aruco_markers_for_chairs_image_2.jpg')]
    objects = pose_estimation.update_final_poses_of_chairs(final_images, objects, poses_of_cameras, aruco_type, camera_calibration_params)
    
    # Return the Poses of Cameras and Objects
    return poses_of_cameras, objects


# Define a Function to get the Chair that has to be Arranged
def get_chair_to_arrange(objects, pose_of_spot_body_frame):

    # Initialise Minimum Distance
    min_distance = 100

    # For every Object in Scene
    for object in objects.objects:

        # If its a Chair
        if "Chair" in object.name:

            # Get the Relative Pose of Chair wrt SPOT Body frame
            pose_of_chair_wrt_spot_frame = np.linalg.inv(pose_of_spot_body_frame) @ object.pose

            # Get the Absolute Distance between Chair and SPOT Robot
            distance_of_chair_from_spot = utils.compute_distance_from_pose(pose_of_chair_wrt_spot_frame)

            # If Current Distance is lesser than Minimum Distance
            if distance_of_chair_from_spot < min_distance:

                # Save Current chair as Chair to be Arranged
                chair_to_arrange = object
                min_distance = distance_of_chair_from_spot
    
    # Return Chair to be Arranged
    return chair_to_arrange


# Define a Function to Arrange Chairs around a Table
def arrange_chairs_around_table(objects, pose_of_spot_body_frame):

    # Until Chairs are Arranged around Table
    print("Arranging Chairs around Table... \n")
    while not objects.is_chairs_arranged():

        # Get the Chair that has to be Arranged around Table
        chair_to_arrange = get_chair_to_arrange(objects, pose_of_spot_body_frame)
        print(chair_to_arrange)
    
    # Display Success Message
    print("Chairs Arranged around Table Successfully!!! \n")


# Define the Main Function
def main():

    # Configure SPOT Robot and make it Stand
    robot = spot_robot_commands.setup_and_configure_robot()
    spot_robot_commands.make_SPOT_stand(robot)

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
            aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()

            # Update the Poses of AruCo markers using Images from both cameras
            objects, pose_of_spot_body_frame = pose_estimation.update_poses_of_objects(
                                                                                            [camera_1_frame, camera_2_frame], 
                                                                                            aruco_tags_data_wrt_spot_frame, 
                                                                                            objects, poses_of_cameras, aruco_type, 
                                                                                            camera_calibration_params
                                                                                      )
            
            # If Chairs are Arranged around Table
            if objects.is_chairs_arranged():
                print("All Chairs are Arranged around Table... \n")

            # Else, Prompt User for Input
            else:
                print("Chairs not Arranged... Press y to Begin Arranging Chairs: \n")
                time.sleep(2)

                # If y Key is Pressed, Begin Arranging
                if cv2.waitKey(1) == ord('y'):             
                    arrange_chairs_around_table(objects, pose_of_spot_body_frame)
        
    # Stop streaming finally
    finally:
        camera_1_pipeline.stop()
        camera_2_pipeline.stop()


# Invoke the Main Function
if __name__ == "__main__":
    main()