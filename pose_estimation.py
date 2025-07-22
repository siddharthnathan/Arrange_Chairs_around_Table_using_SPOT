# Import Necessary Libraries
import numpy as np
import utils
import cv2


# Define a Function to Get Pose of AruCo tag in Image frame
def estimate_poses_of_aruco_tags(frame, objects, aruco_dict_type, camera_calibration_params):

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
            aruco_tag_pose['Name'] = objects.get_key_value_of_object(aruco_id = int(ids[i]), field = "Name")            # Name of AruCo tag
            aruco_tag_pose['Pose'] = utils.compute_pose_from_vectors_or_angles(translation = list(tvecs[i][0]), 
                                                                               rotation = list(rvecs[i][0]), 
                                                                               angle = False)                           # Pose of AruCo tag with Red-square at Top-Left
            aruco_tag_pose['Pose'] = aruco_tag_pose['Pose'] @ np.array([
                                                                            [ 0,  1,  0,  0],
                                                                            [-1,  0,  0,  0],
                                                                            [ 0,  0,  1,  0],
                                                                            [ 0,  0,  0,  1]
                                                                        ])                                              # Pose of AruCo tag with Red-sqaure at Top-Right
                                                                                                                        # since SPOT also estimates pose in same way

            # Draw Pose axes in the AruCo tag image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.1)

            # Append Pose of AruCo tag into List
            poses_of_aruco_tags.append(aruco_tag_pose)

        # Display Image with Estimated Poses of AruCo markers
        cv2.imshow(camera_calibration_params['Name'], frame)
        cv2.waitKey(1)

        # Return AruCo poses
        return poses_of_aruco_tags
    
    # Else Return None
    else:

        # Display Image
        cv2.imshow(camera_calibration_params['Name'], frame)
        cv2.waitKey(1)
        return None


# Define a Function to Compute the Poses of both Cameras wrt Origin
def get_poses_of_cameras(aruco_tags_data_wrt_camera_1_frame, aruco_tags_data_wrt_camera_2_frame):

    # Get the Pose of Origin wrt Camera 1 and Camera 2
    origin_pose_wrt_camera_1 = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Origin')
    origin_pose_wrt_camera_2 = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Origin')

    # Compute the Pose of Camera 1 & Camera 2 wrt Origin
    camera_1_pose = utils.round_matrix_list(np.linalg.inv(origin_pose_wrt_camera_1), 3)
    camera_2_pose = utils.round_matrix_list(np.linalg.inv(origin_pose_wrt_camera_2), 3)

    # Return the Poses of Cameras wrt Origin
    return [camera_1_pose, camera_2_pose]