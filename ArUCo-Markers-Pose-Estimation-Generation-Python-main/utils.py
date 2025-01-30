# Import Necessary Libraries
import cv2


# Define a Function to Display Detected AruCo Marker on an Image
def aruco_display(corners, ids, rejected, image):

	# If there are Corners detected
	if len(corners) > 0:

		# Flatten the ArUco IDs list
		ids = ids.flatten()

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

			# Draw Lines along edges of AruCo Marker
			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

			# Compute and Draw the center (x, y) coordinates of the ArUco marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			# Draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	
	# Return the Image with marked AruCo Tags
	return image