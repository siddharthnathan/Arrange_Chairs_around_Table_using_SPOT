# Import Necessary Libraries
import pyrealsense2 as rs
import numpy as np
import time
import cv2


# Define a Function to Configure and Stream Realsense Pipeline using Cameras
def configure_and_stream_pipeline():

    # Exit if the Number of Cameras are less than 2
    devices = rs.context().devices
    if len(devices) < 2:
        print("Not enough cameras connected.")
        exit()
    
    # If there are 2 cameras connected
    else:

        # Initialise Pipelines for Cameras
        pipelines = []

        # For every Device connected
        for device in devices:

            # Configure the RealSense pipeline
            pipeline = rs.pipeline()
            config = rs.config()

            # Enable the Camera streams
            config.enable_device(device.get_info(rs.camera_info.serial_number))
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            # Start streaming
            pipeline.start(config)
            pipelines.append(pipeline)
        
        # Extract Pipelines for Main camera and Side camera
        camera_1_pipeline, camera_2_pipeline = pipelines

        # Return the Pipelines
        time.sleep(1)
        return camera_1_pipeline, camera_2_pipeline


# Define a Function to Read Camera Calibration parameters
def read_camera_calibration_params():

	# Read the Calibration parameters for both Cameras
	camera_calibration_params = {
									'Camera_1': 
									{
                                        'Name': "Camera_1",
										'Calibration_matrix': np.load('Camera_Calibration/Camera_1/calibration_matrix.npy'),
										'Distortion_coefficients': np.load('Camera_Calibration/Camera_1/distortion_coefficients.npy'),
									},
									'Camera_2': 
									{
                                        'Name': "Camera_2",
										'Calibration_matrix': np.load('Camera_Calibration/Camera_2/calibration_matrix.npy'),
										'Distortion_coefficients': np.load('Camera_Calibration/Camera_2/distortion_coefficients.npy'),
									}
								}
	
	# Return the Camera Calibration parameters
	return camera_calibration_params


# Define a Function to Read Image frames from Pipelines
def read_frame_from_pipeline(camera_pipeline):

    # Get Frames from Main camera
    camera_frame = camera_pipeline.wait_for_frames()
    camera_frame = camera_frame.get_color_frame()
    camera_frame = np.asanyarray(camera_frame.get_data())

    # Return the Image frame
    return camera_frame


# Define a Function to Resize Image frame for given Width
def resize_frame(frame, width):
	
	# Get the Shape of Image frame and maintain Aspect Ratio
	h, w, _ = frame.shape
	height = int(width * (h / w))

	# Resize Image frame and Return 
	frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_CUBIC)
	return frame


# Define a Function to Display Image
def display_image(name, image):
	cv2.imshow(name, image)
	cv2.waitKey(1)