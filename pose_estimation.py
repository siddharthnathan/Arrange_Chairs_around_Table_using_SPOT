# Import Necessary Libraries
import numpy as np
import utils
import cv2


# Define a Function to Get Pose of AruCo tag in Image frame
def get_pose_of_aruco_tags(frame, aruco_dict_type, camera_calibration_params):

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
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.193, matrix_coefficients, distortion_coefficients)
   
            # Store all Parameters into Dictionary
            aruco_tag_pose['Name'] = utils.get_object_with_aruco_tag(int(ids[i]))
            aruco_tag_pose['Pose'] = utils.compute_pose_from_vectors(translation = list(tvecs[i][0]), rotation = list(rvecs[i][0]))
            aruco_tag_pose['Pose'] = aruco_tag_pose['Pose'] @ np.array([
                                                                            [0, 1, 0, 0],
                                                                            [-1, 0, 0, 0],
                                                                            [0, 0, 1, 0],
                                                                            [0, 0, 0, 1]
                                                                        ])

            # Draw Pose axes in the AruCo tag image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.1)

            # Append Pose of AruCo tag into List
            poses_of_aruco_tags.append(aruco_tag_pose)

        # Return Image and AruCo poses
        return frame, poses_of_aruco_tags
    
    # Else Return None
    else:
        return frame, None