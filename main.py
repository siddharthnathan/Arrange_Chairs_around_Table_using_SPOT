# Import Necessary Scripts
import read_video_stream
import pose_estimation
import spot_robot_commands
import coordinate_transformations
import utils

# Import Necessary Libraries
import cv2


# Configure SPOT robot and make it Stand
robot = spot_robot_commands.setup_and_configure_robot()
spot_robot_commands.make_SPOT_stand(robot)
spot_robot_commands.move_robot_to_location(robot)

'''
# Read Image frames continuously
while True:

    # Detect the Fiducials in the Robot's environment
    aruco_tags_data_wrt_spot_body_frame = spot_robot_commands.DetectFiducial(robot).detect_aruco_tags_wrt_spot_body_frame()
    
    # Get the Poses of Chair wrt Camera and SPOT frames
    pose_of_aruco_on_chair_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_body_frame, 'Chair_1')

    # Compute Grasp Pose of Chair wrt SPOT grav aligned body frame
    chair_grasp_pose_wrt_spot = coordinate_transformations.compute_final_grasp_pose_of_chair(pose_of_aruco_on_chair_wrt_spot_frame, offset = 'SPOT')
    grasped = spot_robot_commands.move_arm_to_grasp_chair(robot, chair_grasp_pose_wrt_spot)

    # Check if Chair is Grasped
    if grasped:
    
        # Wait for User input
        ch = input("Press G to begin Grasping: ")
        if ch == 'G':
            continue
    
    # Quit when Q key is Pressed
    if cv2.waitKey(1) == ord('q'):
        break
'''