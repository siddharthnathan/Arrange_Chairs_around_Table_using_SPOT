# Import Necessary Scripts
from coordinate_transformations import Coordinate_frame

# Import Necessary Libraries
import utils
import cv2


# Define a Function to Get Pose of AruCo tags in Image frame
def get_pose_of_aruco_tags(frame, aruco_dict_type, camera_calibration_params):

    # Extract Camera Matrix and Distortion coefficients
    matrix_coefficients, distortion_coefficients = camera_calibration_params['Calibration_matrix'], camera_calibration_params['Distortion_coefficients']

    # Convert the Image into Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Create the Parameters to Detect AruCo markers
    arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    arucoParams = cv2.aruco.DetectorParameters()

    # Adjust detector parameters for small markers
    arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # Create AruCo detector with those Parameters
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    
    # Create the Parameters to Detect AruCo markers
    corners, ids, _ = detector.detectMarkers(gray)

    # If AruCo IDs are Detected
    if ids is not None:

        # Initialise List to store Data of AruCo markers
        aruco_tags_data = []
    
        # For every AruCo ID detected
        for i in range(len(ids)):

            # Compute the Pose of AruCo markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, utils.get_size_of_aruco_tag(int(ids[i])), matrix_coefficients, distortion_coefficients)
   
            # Get the Name, Translation vector and Rotation angles of AruCo tag
            aruco_tag_name = utils.get_object_with_aruco_tag(int(ids[i]))
            aruco_tag_translation_vector = utils.round_float_list(list(tvecs[i][0]), 3)
            aruco_tag_rotation_angles = utils.convert_rotation_vector_to_angles(list(rvecs[i][0]))

            # Draw Pose axes in the AruCo tag image
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.1)

            # Store data in a AruCo tag object
            aruco_tag_frame = Coordinate_frame(aruco_tag_name, aruco_tag_translation_vector, aruco_tag_rotation_angles)

            # Append Pose of AruCo tag into List
            aruco_tags_data.append(aruco_tag_frame)
        
        # Return Image and AruCo poses
        return frame, aruco_tags_data
    
    # Else Return None
    else:
        return frame, None