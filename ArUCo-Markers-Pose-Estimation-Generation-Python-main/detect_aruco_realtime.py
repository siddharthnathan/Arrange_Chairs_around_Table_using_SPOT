# Import Necessary Libraries
from utils import aruco_display
import cv2


# Define Webcam Object to Read Video stream
cam = cv2.VideoCapture(0)

# Until the Camera stream exists
while True:

	# Read the Frame from Camera stream
	ret, frame = cam.read()
    
	# If Frame exists
	if ret:
		
		# Get the Shape of Frame and Resize it
		h, w, _ = frame.shape
		width = 600
		height = int(width * (h / w))
		frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_CUBIC)

		# Create the Parameters to Detect AruCo markers
		arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
		arucoParams =  cv2.aruco.DetectorParameters()
		detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

		# Detect Corners, IDs of AruCo markers in Frame
		markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
		corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters = arucoParams)

		# Display the AruCo marker on the Frame
		detected_markers = aruco_display(corners, ids, rejected, frame)
		cv2.imshow("Image", detected_markers)

		# Check if Escape Key is Pressed
		k = cv2.waitKey(1)
		if k % 256 == 27:
			print("Escape hit, closing...")
			break

# Release Camera object and Destroy all Windows
cam.release()
cv2.destroyAllWindows()