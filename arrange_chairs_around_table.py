# Import Necessary Scripts
import spot_robot_commands
import read_video_stream
import pose_estimation
import utils

# Import Necessary Libraries
import numpy as np
import time
import cv2


# Define a Function to get the Chair that has to be Arranged
def get_chair_to_arrange(unarranged_chairs, pose_of_spot_body_frame):

    # Initialise Minimum Distance
    min_distance = 100

    # For every Unarranged Chair in Scene
    for chair in unarranged_chairs:

        # Get the Relative Pose of Chair wrt SPOT Body frame
        pose_of_chair_wrt_spot_frame = utils.round_matrix_list(np.linalg.inv(pose_of_spot_body_frame) @ chair.pose['Pose'], 3)

        # Get the Absolute Distance between Chair and SPOT Robot
        distance_of_chair_from_spot = utils.compute_distance_from_pose(pose_of_chair_wrt_spot_frame)

        # If Current Distance is lesser than Minimum Distance
        if distance_of_chair_from_spot < min_distance:

            # Save Current chair as Chair to be Arranged
            chair_to_arrange = chair
            min_distance = distance_of_chair_from_spot
    
    # Return Chair to be Arranged
    return chair_to_arrange


# Define a Function to Let go of Chair after Arranging around Table
def let_go_of_chair(robot):

    # Unfreeze Arm Joints
    spot_robot_commands.unfreeze_arm_joints(robot)

    # Open Arm Gripper
    spot_robot_commands.open_or_close_gripper(robot, action = 'open')

    # Bring Arm back to Default Pose
    spot_robot_commands.move_arm_to_default_pose(robot)

    # Close Arm Gripper
    spot_robot_commands.open_or_close_gripper(robot, action = 'close')


# Define a Function to Arrange Chair around Table
def arrange_chair_around_table(robot, objects, pose_of_spot_body_frame, chair_to_arrange):

    # Get the Pose of AruCo tags wrt SPOT Body Frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, chair_to_arrange.aruco_id)

    # Move Robot behind Chair
    spot_robot_commands.move_SPOT_behind_chair(robot, pose_of_chair_wrt_spot)
        
    # Get the Pose of SPOT Body Frame and Pose of Chair wrt SPOT frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_spot_body_frame = pose_estimation.localize_spot_wrt_origin(aruco_tags_data_wrt_spot_frame, objects)
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, chair_to_arrange.aruco_id)

    # Compute the Pose of Chair wrt Origin
    current_chair_pose = utils.round_matrix_list(pose_of_spot_body_frame @ pose_of_chair_wrt_spot, 3)
    chair_to_arrange.pose['Pose'] = current_chair_pose
    chair_to_arrange.pose['Translation'], chair_to_arrange.pose['Rotation'] = utils.get_components_from_pose_for_chair(current_chair_pose)
    
    # Grasp Chair using Robot
    spot_robot_commands.grasp_chair_using_SPOT(robot, pose_of_chair_wrt_spot)
    time.sleep(3)

    # Convert the Current and Final pose of Chair wrt SPOT frame
    current_chair_pose_wrt_spot = chair_to_arrange.pose['Pose'] @ np.array([
                                                                                [ 0,     -1,    0,      0],
                                                                                [ 0.329,  0,    0.944,  0],
                                                                                [-0.944,  0,    0.329,  0],
                                                                                [ 0,      0,    0,      1]
                                                                           ])
    final_chair_pose_wrt_spot = chair_to_arrange.final_pose['Pose'] @ np.array([
                                                                                    [ 0,     -1,    0,      0],
                                                                                    [ 0.329,  0,    0.944,  0],
                                                                                    [-0.944,  0,    0.329,  0],
                                                                                    [ 0,      0,    0,      1]
                                                                               ])
    
    # Compute Pose to move SPOT to Arrange Chair
    pose_to_move_chair = utils.round_matrix_list(np.linalg.inv(current_chair_pose_wrt_spot) @ final_chair_pose_wrt_spot, num_decimal_places = 3)
    translation, rotation = utils.get_components_from_pose(pose_to_move_chair)
    x, y, _ = translation
    _, _, rz = rotation

    # Move SPOT to Arrange Chair
    print("Moving SPOT by ", [x, y, rz])
    spot_robot_commands.move_robot_to_location(robot, pose_to_move_chair)
    
    # Let go of the Chair after Arranging
    let_go_of_chair(robot)

    # Go to the Nearest Waypoint from the Unarranged Chair
    spot_robot_commands.move_SPOT_to_nearest_waypoint(robot, objects)
    

# Define a Function to Arrange Chairs around a Table
def arrange_chairs_around_table(robot, objects, unarranged_chairs, pose_of_spot_body_frame):
    
    # Until Chairs are Arranged around Table
    print("Arranging Chairs around Table... \n")

    # Get the Chair that has to be Arranged around Table
    #chair_to_arrange = get_chair_to_arrange(unarranged_chairs, pose_of_spot_body_frame)
    chair_to_arrange = objects.chairs[1]

    # Go to the Nearest Waypoint from the Pose of 
    nearest_waypoint = spot_robot_commands.move_SPOT_to_nearest_waypoint(robot, objects)
    
    # Find the Nearest Waypoint from the Unarranged Chair
    if chair_to_arrange.name == 'Chair_1' or chair_to_arrange.name == 'Chair_2':
        destination_waypoint = objects.waypoints[6]
    elif chair_to_arrange.name == 'Chair_3' or chair_to_arrange.name == 'Chair_4':
        destination_waypoint = objects.waypoints[2]

    # Get the Path for the SPOT Robot across Waypoints
    waypoints_path = utils.get_waypoints_path(nearest_waypoint, destination_waypoint)

    # Move Robot across Waypoints
    spot_robot_commands.move_SPOT_across_waypoints(robot, objects, waypoints_path)

    # Arrange the Chair that has to be Arranged
    print("Arranging ", chair_to_arrange.name)
    time.sleep(2)
    arrange_chair_around_table(robot, objects, pose_of_spot_body_frame, chair_to_arrange)
    


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

    # Initialise Objects poses
    objects = utils.Objects()
    
    # Try block
    try:

        # Read Image frames continuously
        while True:

            # Read Image frames from both Cameras
            camera_1_frame = read_video_stream.read_frame_from_pipeline(camera_1_pipeline)
            camera_2_frame = read_video_stream.read_frame_from_pipeline(camera_2_pipeline)

            # Get the Pose of AruCo tags wrt SPOT Body Frame
            aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
            
            # Determine the Pose of SPOT body frame wrt Origin AruCo tag
            pose_of_spot_body_frame = pose_estimation.localize_spot_wrt_origin(aruco_tags_data_wrt_spot_frame, objects)

            # Update the Poses of Objects using Images from both cameras
            objects = pose_estimation.update_poses_of_chairs(
                                                                [camera_1_frame, camera_2_frame], objects,
                                                                aruco_tags_data_wrt_spot_frame, 
                                                                pose_of_spot_body_frame,
                                                                aruco_type, camera_calibration_params
                                                            )
            
            # Get Unarranged chairs around Table
            unarranged_chairs = objects.get_unarranged_chairs()

            # If all Chairs are Arranged around Table
            if len(unarranged_chairs) == 0:
                print("All Chairs are Arranged around Table... \n")

            # Else, Begin Arranging Chairs   
            else:
                arrange_chairs_around_table(robot, objects, unarranged_chairs, pose_of_spot_body_frame)

            break
            # Wait 3 seconds
            time.sleep(3)        

    # Stop streaming finally
    finally:
        camera_1_pipeline.stop()
        camera_2_pipeline.stop()
    

# Invoke the Main Function
if __name__ == "__main__":
    main()