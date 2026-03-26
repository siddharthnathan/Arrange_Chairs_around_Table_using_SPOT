import spot_robot_commands
import pose_estimation
import numpy as np
import utils

# Configure SPOT Robot and make it Stand
robot = spot_robot_commands.setup_and_configure_robot()
spot_robot_commands.make_SPOT_stand(robot)

# Initialise Objects poses
objects = utils.Objects()


# Get the Pose of SPOT Body Frame and Pose of Chair wrt SPOT frame
aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 9)

spot_robot_commands.move_SPOT_behind_chair(robot, pose_of_chair_wrt_spot)

# Grasp Chair using Robot
aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, 9)
spot_robot_commands.grasp_chair_using_SPOT(robot, pose_of_chair_wrt_spot)

# Compute Final Pose of Chair wrt SPOT
pose_to_move_chair = np.array([
                                    [1, 0, 0, 0.0],
                                    [0, 1, 0, 0.0],
                                    [0, 0, 1, 0.1],
                                    [0, 0, 0, 1.0],
                              ])
final_grasp_pose_of_chair_wrt_spot = utils.round_matrix_list(spot_robot_commands.get_pose_of_arm(robot) @ pose_to_move_chair, 3)

# Move SPOT to Arrange Chair
spot_robot_commands.move_arm_to_grasp_pose(robot, final_grasp_pose_of_chair_wrt_spot)

# Let go of the Chair after Arranging
spot_robot_commands.let_go_of_chair(robot)
