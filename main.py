# Import Necessary Scripts
import spot_robot_commands
import utils
import numpy as np
import math


# Define the Main Function
def main():

    # Configure SPOT Robot and make it Stand
    robot = spot_robot_commands.setup_and_configure_robot()
    spot_robot_commands.make_SPOT_stand(robot)

    # Get the Object Mapping with AruCo IDs
    object_mapping = utils.read_mapping_of_objects()
    
    # Get the Pose of AruCo tags wrt SPOT Body Frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, object_mapping[7])

    # Move Robot behind Chair
    spot_robot_commands.move_SPOT_behind_chair(robot, pose_of_chair_wrt_spot)
    
    # Get the Pose of AruCo tags wrt SPOT Body Frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, object_mapping[7])

    # Grasp Chair using Robot
    spot_robot_commands.grasp_chair_using_SPOT(robot, pose_of_chair_wrt_spot)
    
    # Move Chair to given Pose
    c, s = math.cos(np.deg2rad(0)), math.sin(np.deg2rad(0))
    pose = np.array([
                        [ c,  s,  0,  0.10],
                        [-s,  c,  0,  0.00],
                        [ 0,  0,  1,  0.00],
                        [ 0,  0,  0,  1.00]
                    ])
    spot_robot_commands.move_robot_to_location(robot, pose)    
    
    spot_robot_commands.unfreeze_arm_joints(robot)
    spot_robot_commands.open_or_close_gripper(robot, action = 'open')
    spot_robot_commands.move_arm_to_default_pose(robot)
    spot_robot_commands.open_or_close_gripper(robot, action = 'close')
    
    # Move SPOT back 1m
    pose = np.array([
                        [ 1,  0,  0, -1.00],
                        [ 0,  1,  0,  0.00],
                        [ 0,  0,  1,  0.00],
                        [ 0,  0,  0,  1.00]
                    ])
    spot_robot_commands.move_robot_to_location(robot, pose)
    

# Invoke the Main Function
if __name__ == "__main__":
    main()