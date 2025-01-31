# Import Necessary Libraries
import cv2


# Define a Function to Resize Image frame for given Width
def resize_frame(frame, width):
	
	# Get the Shape of Image frame and maintain Aspect Ratio
	h, w, _ = frame.shape
	height = int(width * (h / w))

	# Resize Image frame and Return 
	frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_CUBIC)
	return frame