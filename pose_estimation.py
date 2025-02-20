# Import Necessary Libraries
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
    
    # Create the Parameters to Detect AruCo markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters = arucoParams)

    # If AruCo tags are Detected
    if len(corners) > 0:

        # Initialise List to store Pose of AruCo tags
        poses_of_aruco_tags = []

        # For every Detected AruCo tag
        for i in range(0, len(ids)):

            # Estimate pose of each marker and return the values of Rotational vector & Translation vector
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            
            # Create a Dictionary to store Pose of AruCo tag and Append
            aruco_tag_pose = {}
            aruco_tag_pose['ID'] = int(ids[i])
            aruco_tag_pose['Translation'] = tvec[0][0]
            aruco_tag_pose['Rotation'] = rvec[0][0]
            poses_of_aruco_tags.append(aruco_tag_pose)

            # Draw a square around the AruCo tag in Frame
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis at the center of the AruCo tag in Frame
            frame_with_aruco_tags_pose = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, length = 3e-3)

        # Return the Image frame with Pose of AruCo markers
        return frame_with_aruco_tags_pose, poses_of_aruco_tags

    # Else when AruCo tags are not Detected
    else:
        return frame, None