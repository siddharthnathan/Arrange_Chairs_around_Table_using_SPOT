# Import Necessary Libraries
import read_video_stream
import numpy as np
import utils
import cv2


# Define the Pose of Grasp location (Seatrest from above: X - down, Y - left) wrt AruCo on Chair (Seatrest behind chair: X - right, Y - up)
pose_of_grasp_location_wrt_aruco_on_chair = np.array([
                                                        [ 0, -1,  0,  0.1000],
                                                        [-1,  0,  0,  0.3000],
                                                        [ 0,  0, -1, -0.0500],
                                                        [ 0,  0,  0,  1.0000]
                                                    ])


# Define a Function to Account for the Offset in Grasp pose wrt SPOT
def account_for_offset(grasp_pose_wrt_spot, offset):

    # Get the Translation and Rotation of Grasp pose wrt SPOT
    translation, rotation = utils.get_translation_and_rotation_from_pose(grasp_pose_wrt_spot, angle = True)
    
    # If Offset is due to Camera
    if offset == 'SPOT':

        # Update Translation according to the Offsets in m
        translation[0] -= 0.08
        translation[1] -= 0.10
        translation[2] += 0.01

        # Update Rotation accordingly
        rotation[0] = 0
        rotation[1] = 90
        rotation[2] = 0
    
    # If Offset is due to SPOT
    elif offset == 'Camera':

        # Update Translation according to the Offsets in m
        translation[0] += 0
        translation[1] += 0
        translation[2] += 0

        # Update Rotation accordingly
        rotation[0] = 0
        rotation[1] = 90
        rotation[2] = 0

    # Update Pose accordingly and Return
    grasp_pose_wrt_spot = utils.compute_pose_from_vectors_or_angles(translation, rotation, angle = True)
    return grasp_pose_wrt_spot


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
            aruco_tag_pose['ID'] = int(ids[i])                                                                          # ID of AruCo tag
            aruco_tag_pose['Pose'] = utils.compute_pose_from_vectors_or_angles(
                                                                                translation = list(tvecs[i][0]), 
                                                                                rotation = list(rvecs[i][0]), 
                                                                                angle = False
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


# Define a Function to Compute the Poses of both Cameras wrt Origin
def get_poses_of_cameras(initial_images, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(initial_images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(initial_images[1], aruco_type, camera_calibration_params['Camera_2'])

    # Get the Pose of Origin wrt Camera 1 and Camera 2
    origin_pose_wrt_camera_1 = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, 'Origin')
    origin_pose_wrt_camera_2 = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, 'Origin')

    # Compute the Pose of Camera 1 & Camera 2 wrt Origin
    camera_1_pose = utils.round_matrix_list(np.linalg.inv(origin_pose_wrt_camera_1), 3)
    camera_2_pose = utils.round_matrix_list(np.linalg.inv(origin_pose_wrt_camera_2), 3)

    # Return the Poses of Cameras wrt Origin
    return [camera_1_pose, camera_2_pose]


# Define a Function to Update Pose of Origins in scene
def update_poses_of_origins(initial_images, objects, poses_of_cameras, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(initial_images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(initial_images[1], aruco_type, camera_calibration_params['Camera_2'])

    # For every Object in Objects
    for i in range(len(objects.objects)):

        # If Object is not Origin nor Chair
        if objects.objects[i].name != "Origin" and "Chair" not in objects.objects[i].name:

            # Get the Pose of Object wrt Camera frames
            pose_of_object_wrt_camera_1_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, objects.objects[i].name)
            pose_of_object_wrt_camera_2_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, objects.objects[i].name)

            # If Object is detected by Camera 1
            if pose_of_object_wrt_camera_1_frame is not None:

                # Update Pose of Object wrt Origin AruCo tag
                objects.objects[i].final_pose = poses_of_cameras[0] @ pose_of_object_wrt_camera_1_frame
            
            # If Object is detected by Camera 2
            else:

                # Update Pose of Object wrt Origin AruCo tag
                objects.objects[i].final_pose = poses_of_cameras[1] @ pose_of_object_wrt_camera_2_frame

    # Return the Objects with updated Poses
    return objects


# Define a Function to Update Final Pose of Chairs in scene
def update_final_poses_of_chairs(images, objects, poses_of_cameras, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(images[1], aruco_type, camera_calibration_params['Camera_2'])

    # For every Object in Objects
    for i in range(len(objects.objects)):

        # If Object is Chair
        if "Chair" in objects.objects[i].name:

            # Get the Pose of Chair wrt Camera frames
            pose_of_chair_wrt_camera_1_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, objects.objects[i].name)
            pose_of_chair_wrt_camera_2_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, objects.objects[i].name)

            # If Chair is detected by Camera 1
            if pose_of_chair_wrt_camera_1_frame is not None:

                # Update Final Pose of Chair wrt Origin AruCo tag
                objects.objects[i].final_pose = poses_of_cameras[0] @ pose_of_chair_wrt_camera_1_frame
            
            # If Chair is detected by Camera 2
            else:

                # Update Final Pose of Chair wrt Origin AruCo tag
                objects.objects[i].final_pose = poses_of_cameras[1] @ pose_of_chair_wrt_camera_2_frame

    # Return the Objects with updated Poses
    return objects


# Define a Function to Localize SPOT wrt Origin AruCo tag
def localize_spot_wrt_origin(aruco_tags_data_wrt_spot_frame, objects):

    # For every AruCo tag detected by SPOT
    for aruco_tag_data_wrt_spot_frame in aruco_tags_data_wrt_spot_frame:
        
        # Get the AruCo ID detected
        aruco_id = aruco_tag_data_wrt_spot_frame['ID']

        # Get the Oject name using AruCo ID
        object_name = objects.objects_with_aruco_ids[aruco_id]

        # If Object is Origin
        if object_name == "Origin":

            # Return the Pose of SPOT body frame wrt Origin
            return utils.round_matrix_list(np.linalg.inv(aruco_tag_data_wrt_spot_frame['Pose']), 3)

        # Else if Object is not Chair
        elif "Chair" not in object_name:
            
            # Get the Pose of Non-Origin marker wrt Origin marker
            pose_of_object = objects.get_value_of_object_using_key(aruco_id, "Final_Pose")

            # Return the Pose of SPOT body frame wrt Origin
            return pose_of_object @ utils.round_matrix_list(np.linalg.inv(aruco_tag_data_wrt_spot_frame['Pose']), 3)


# Define a Function to Update Poses of Objects wrt Camera frame and SPOT frame
def update_poses_of_objects(images, aruco_tags_data_wrt_spot_frame, objects, poses_of_cameras, aruco_type, camera_calibration_params):

    # Get the Pose of AruCo tags wrt both Cameras
    aruco_tags_data_wrt_camera_1_frame = estimate_poses_of_aruco_tags(images[0], aruco_type, camera_calibration_params['Camera_1']) 
    aruco_tags_data_wrt_camera_2_frame = estimate_poses_of_aruco_tags(images[1], aruco_type, camera_calibration_params['Camera_2'])

    # Determine the Pose of SPOT body frame wrt Origin AruCo tag
    pose_of_spot_body_frame = localize_spot_wrt_origin(aruco_tags_data_wrt_spot_frame, objects)

    # For every Object in Objects
    for i in range(len(objects.objects)):

        # If Object is a Chair
        if "Chair" in objects.objects[i].name:

            # Get the Pose of Object wrt Camera and SPOT frames
            pose_of_object_wrt_camera_1_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_1_frame, objects.objects[i].name)
            pose_of_object_wrt_camera_2_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_camera_2_frame, objects.objects[i].name)
            pose_of_object_wrt_spot_frame = utils.get_pose_of_aruco_tag(aruco_tags_data_wrt_spot_frame, objects.objects[i].name)

            # If Object is detected by Camera 1
            if pose_of_object_wrt_camera_1_frame is not None:

                # Update Pose of Object wrt Origin AruCo tag
                objects.objects[i].pose = poses_of_cameras[0] @ pose_of_object_wrt_camera_1_frame
            
            # If Object is detected by Camera 2
            elif pose_of_object_wrt_camera_2_frame is not None:

                # Update Pose of Object wrt Origin AruCo tag
                objects.objects[i].pose = poses_of_cameras[1] @ pose_of_object_wrt_camera_2_frame
            
            # If Object is detected by SPOT
            elif pose_of_object_wrt_spot_frame is not None:
                
                # Update Pose of Object wrt Origin AruCo tag
                objects.objects[i].pose = pose_of_spot_body_frame @ pose_of_object_wrt_spot_frame
            
            # Else if Object is not Detected
            else:
                objects.objects[i].pose = None

    # Return the Objects with updated Poses alongwith SPOT body frame pose
    return objects, pose_of_spot_body_frame