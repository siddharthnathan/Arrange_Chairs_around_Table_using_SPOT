# Import Necessary Libraries
import cv2


#detected_aruco_tags = aruco_tag_detection.detect_aruco_markers_in_frame(frame, camera, aruco_type = cv2.aruco.DICT_APRILTAG_36h11)

# Define a Function ot Detect AruCo markers in Image frame
def detect_aruco_markers_in_frame(frame, camera, aruco_type):

	# Create the Parameters to Detect AruCo markers
	arucoDict = cv2.aruco.getPredefinedDictionary(aruco_type)
	arucoParams =  cv2.aruco.DetectorParameters()

	# Detect Corners, IDs of AruCo markers in Frame
	corners, ids, _ = cv2.aruco.detectMarkers(frame, arucoDict, parameters = arucoParams)

	# If AruCo tags are Detected
	if ids is not None:

		# Store the Corners and IDs of Detected AruCo tags as Dictionary
		detected_aruco_tags = get_dict_of_aruco_tags_detected(corners, ids)

		# Display the AruCo marker on the Frame
		aruco_in_image = aruco_display(detected_aruco_tags, frame)
		cv2.imshow(camera, aruco_in_image)

		# Return the Dictionary of AruCo tags
		return detected_aruco_tags

	# Else return None if No AruCo tags are Detected
	else:
		cv2.imshow(camera, frame)
		return None


# Define a Function to Store the Corners and IDs of Detected AruCo tags as Dictionary
def get_dict_of_aruco_tags_detected(corners, ids):

	# Initialise List of Dictionary to store AruCo Tags detected
	aruco_tags_dict = []

	# Loop over the detected ArUCo corners
	for (markerCorner, markerID) in zip(corners, ids):

		# Extract the marker corners (which are always returned in
		# Top-left, Top-right, Bottom-right, and Bottom-left order)
		corners = markerCorner.reshape((4, 2))
		(topLeft, topRight, bottomRight, bottomLeft) = corners

		# Convert each of the (x, y) coordinate pairs to integers
		topRight = (int(topRight[0]), int(topRight[1]))
		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
		topLeft = (int(topLeft[0]), int(topLeft[1]))

		# Compute the Centre of AruCo tags detected
		center_x = int((topLeft[0] + bottomRight[0]) / 2.0)
		center_y = int((topLeft[1] + bottomRight[1]) / 2.0)

		# Store the ID, Corners, Center of AruCo tag into Dictionary and Append into List
		aruco_tag = {}
		aruco_tag['ID'] = int(markerID)
		aruco_tag['Corners'] = [topLeft, topRight, bottomLeft, bottomRight]
		aruco_tag['Center'] = [center_x, center_y]
		aruco_tags_dict.append(aruco_tag)
	
	# Return the List of AruCo tags detected
	return aruco_tags_dict


# Define a Function to Display Detected AruCo Tags on an Image frame
def aruco_display(aruco_tags_dict, image):

	# For every AruCo tag Detected
	for aruco_tag in aruco_tags_dict:
		
		# Get the Corners of AruCo tag
		[topLeft, topRight, bottomLeft, bottomRight] = aruco_tag['Corners']

		# Draw Lines along edges of AruCo Marker
		cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
		cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
		cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
		cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

		# Draw the center (x, y) coordinates of the ArUco marker
		[center_x, center_y] = aruco_tag['Center']
		cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)
		
		# Draw the ArUco marker ID on the image
		cv2.putText(image, str(aruco_tag['ID']),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	
	# Return the Image with marked AruCo Tags
	return image