# Import Necessary Scripts
import spot_robot_commands
import utils
import numpy as np


# Define the Main Function
def main():

    # Configure SPOT Robot and make it Stand
    robot = spot_robot_commands.setup_and_configure_robot()
    spot_robot_commands.make_SPOT_stand(robot)

    # Get the Object Mapping with AruCo IDs
    object_mapping = utils.read_mapping_of_objects()
    
    # Get the Pose of AruCo tags wrt SPOT Body Frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, object_mapping[9])

    # Move Robot behind Chair
    spot_robot_commands.move_SPOT_behind_chair(robot, pose_of_chair_wrt_spot)
    
    # Get the Pose of AruCo tags wrt SPOT Body Frame
    aruco_tags_data_wrt_spot_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    pose_of_chair_wrt_spot = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, object_mapping[9])

    # Grasp Chair using Robot
    spot_robot_commands.grasp_chair_using_SPOT(robot, pose_of_chair_wrt_spot)


# Invoke the Main Function
if __name__ == "__main__":
    main()