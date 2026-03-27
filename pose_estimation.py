# Import Necessary Libraries
import spot_robot_commands
import read_video_stream
import numpy as np
import utils
import cv2


# Define the Pose of Grasp location (Seatrest from above: X - down, Y - left) wrt AruCo on Chair (Seatrest behind chair: X - right, Y - up)
grasp_pose_location_wrt_aruco_on_chair = np.array([
                                                    [ 0, -1,  0,  0.0000],
                                                    [-1,  0,  0,  0.4000],
                                                    [ 0,  0, -1, -0.0100],
                                                    [ 0,  0,  0,  1.0000]
                                                 ])


# Define a Function to Get Pose of AruCo tag in Image frame
def estimate_poses_of_aruco_tags(frame, aruco_dict_type, camera_calibration_params):

    # Extract Camera Matrix and Distortion coefficients
    matrix_coefficients, distortion_coefficients = camera_calibration_params['Calibration_matrix'], camera_calibration_params['Distortion_coefficients']

    # Convert the Image into Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Create the Parameters to Detect AruCo markers
    arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    arucoParams =  cv2.aruco.DetectorParameters()

    # Adjust detector parameters for small markers
    arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # Create AruCo detector with those Parameters
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    
    # Create the Parameters to Detect AruCo markers
    corners, ids, _ = detector.detectMarkers(gray)

    # If AruCo IDs are Detected
    if ids is not None:

        # Initialise List to store Poses of AruCo markers
        poses_of_aruco_tags = []
    
        # For every AruCo ID detected
        for i in range(len(ids)):

            # Initialise Dictionary to store Pose of an AruCo marker
            aruco_tag_pose = {}

            # Compute the Pose of AruCo markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.20, matrix_coefficients, distortion_coefficients)
   
            # Store all Parameters into Dictionary
            aruco_tag_pose['ID'] = int(ids[i])                                                                   # ID of AruCo tag
            aruco_tag_pose['Pose'] = utils.compute_pose_from_components(
                                                                            translation = list(tvecs[i][0]), 
                                                                            rotation = list(rvecs[i][0]), 
                                                                            degrees = False
                                                                       )                                         # Pose of AruCo tag with Red-square at Top-Left
           
            # Draw Pose axes in the AruCo tag image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.1)

            # Append Pose of AruCo tag into List
            poses_of_aruco_tags.append(aruco_tag_pose)

        # Display Image with Estimated Poses of AruCo markers
        read_video_stream.display_image(camera_calibration_params['Name'], frame)

        # Return Estimated Poses of AruCo tags
        return poses_of_aruco_tags
    
    # Else Return None
    else:

        # Display Image
        read_video_stream.display_image(camera_calibration_params['Name'], frame)
        return None


# Define a Function to Localize SPOT wrt Origin AruCo tag
def localize_spot_wrt_origin(aruco_tags_data_wrt_spot_frame, objects):

    # For every AruCo tag detected by SPOT
    for aruco_tag_data_wrt_spot_frame in aruco_tags_data_wrt_spot_frame:
        
        # Get the AruCo ID detected
        aruco_id = aruco_tag_data_wrt_spot_frame['ID']

        # Get the Oject name using AruCo ID
        object_name = objects.get_name_of_object(aruco_id)

        # If Object is Wall marker
        if "Wall" in object_name:
            
            # Get the Pose of Non-Origin marker wrt Origin marker
            pose_of_wall = objects.get_pose_of_object(object_name)

            # Return the Pose of SPOT body frame wrt Origin
            return utils.round_matrix_list(pose_of_wall @ np.linalg.inv(aruco_tag_data_wrt_spot_frame['Pose']), 3)


# Define a Function to Update Poses of Chairs
def update_poses_of_chairs(images, objects, aruco_tags_data_wrt_spot_frame, 
                           pose_of_spot_body_frame, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(images[1], aruco_type, camera_calibration_params['Camera_2'])
    
    # For every Chair Object
    for i in range(len(objects.chairs)):

        # Get the Pose of Chair wrt Camera and SPOT frames
        pose_of_chair_wrt_camera_1_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, objects.chairs[i].aruco_id)
        pose_of_chair_wrt_camera_2_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, objects.chairs[i].aruco_id)
        pose_of_chair_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, objects.chairs[i].aruco_id)

        # If Chair is detected by Camera 1
        if pose_of_chair_wrt_camera_1_frame is not None:

            # Update Pose of Chair wrt Origin AruCo tag
            objects.chairs[i].pose['Pose'] = utils.round_matrix_list(objects.get_pose_of_object('Camera_1') @ pose_of_chair_wrt_camera_1_frame, 3)

            # Update Pose of Chair wrt Camera frame
            objects.chairs[i].pose_wrt_camera['Pose'] = pose_of_chair_wrt_camera_1_frame
        
        # If Chair is detected by Camera 2
        elif pose_of_chair_wrt_camera_2_frame is not None:

            # Update Pose of Chair wrt Origin AruCo tag
            objects.chairs[i].pose['Pose'] = utils.round_matrix_list(objects.get_pose_of_object('Camera_2') @ pose_of_chair_wrt_camera_2_frame, 3)

            # Update Pose of Chair wrt Camera frame
            objects.chairs[i].pose_wrt_camera['Pose'] = pose_of_chair_wrt_camera_2_frame

        # If Chair is detected by SPOT
        elif pose_of_chair_wrt_spot_frame is not None:

            # Update Pose of Chair wrt Origin AruCo tag
            objects.chairs[i].pose['Pose'] = utils.round_matrix_list(pose_of_spot_body_frame @ pose_of_chair_wrt_spot_frame, 3)

        # Update pose components of Chairs
        objects.chairs[i].pose['Translation'], objects.chairs[i].pose['Rotation'] = utils.get_components_from_pose_for_chair(objects.chairs[i].pose['Pose'])
        objects.chairs[i].pose_wrt_camera['Translation'], objects.chairs[i].pose_wrt_camera['Rotation'] = utils.get_components_from_pose(objects.chairs[i].pose_wrt_camera['Pose'])

    # Return the Objects with updated Poses
    return objects


# Define a Function to Update Final poses of Chairs wrt Camera Frame
def update_final_poses_of_chairs_wrt_camera(images, objects, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(images[1], aruco_type, camera_calibration_params['Camera_2'])
    
    # For every Chair Object
    for i in range(len(objects.chairs)):

        # Get the Pose of Chair wrt Camera frame
        pose_of_chair_wrt_camera_1_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, objects.chairs[i].aruco_id)
        pose_of_chair_wrt_camera_2_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, objects.chairs[i].aruco_id)

        # If Chair is detected by Camera 1
        if pose_of_chair_wrt_camera_1_frame is not None:

            # Update Pose of Chair wrt Camera
            objects.chairs[i].final_pose_wrt_camera['Pose'] = pose_of_chair_wrt_camera_1_frame
        
        # If Chair is detected by Camera 2
        elif pose_of_chair_wrt_camera_2_frame is not None:

            # Update Pose of Chair wrt Camera
            objects.chairs[i].final_pose_wrt_camera['Pose'] = pose_of_chair_wrt_camera_2_frame
        
        # Update pose components of Chairs
        objects.chairs[i].final_pose_wrt_camera['Translation'], objects.chairs[i].final_pose_wrt_camera['Rotation'] = utils.get_components_from_pose(objects.chairs[i].final_pose_wrt_camera['Pose'])
    
    # Return the Objects with updated Poses
    return objects


# Define a Function to Compute Grasp Pose to Grasp Chair
def compute_grasp_pose(pose_of_chair_wrt_spot):

    # Extract Translation vector for AruCo marker on Chair wrt SPOT
    x, y, z = pose_of_chair_wrt_spot[0][3], pose_of_chair_wrt_spot[1][3], pose_of_chair_wrt_spot[2][3]
    
    # Update Pose of AruCo on Chair wrt SPOT Robot
    pose_of_chair_wrt_spot = np.array([
                                            [ 0,  0, -1,  x],
                                            [-1,  0,  0,  y],
                                            [ 0,  1,  0,  z],
                                            [ 0,  0,  0,  1]
                                      ])
    
    # Compute and Return the Grasp Pose of Chair wrt SPOT
    grasp_pose_wrt_spot = utils.round_matrix_list(pose_of_chair_wrt_spot @ grasp_pose_location_wrt_aruco_on_chair, 3)
    return grasp_pose_wrt_spot


# Define a Function to Get the Pose of ArUco on Chair wrt SPOT using Grasp pose
def get_pose_of_chair_wrt_SPOT(robot):
    return utils.round_matrix_list(spot_robot_commands.get_pose_of_arm(robot) @ np.linalg.inv(grasp_pose_location_wrt_aruco_on_chair), 3)