# Import Necessary Libraries
import pyrealsense2 as rs
import numpy as np
import cv2


# Define a Function to Configure and Stream Realsense Pipeline
def configure_and_stream_pipeline():

    # Configure the RealSense pipeline
    pipeline1 = rs.pipeline()
    pipeline2 = rs.pipeline()
    config1 = rs.config()
    config2 = rs.config()

    # Find the cameras devices
    devices = rs.context().devices
    if len(devices) < 2:
        print("Not enough cameras connected.")
        exit()

    # Enable the Camera streams
    config1.enable_device(devices[0].get_info(rs.camera_info.serial_number))
    config2.enable_device(devices[1].get_info(rs.camera_info.serial_number))
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline1.start(config1)
    pipeline2.start(config2)

    # Return the Pipelines
    return pipeline1, pipeline2


# Define a Function to Read Image frames from Pipelines
def read_frames_from_pipelines(main_cam_pipeline, side_cam_pipeline):

    # Get Frames from Main camera
    main_camera_frame = main_cam_pipeline.wait_for_frames()
    main_camera_frame = main_camera_frame.get_color_frame()
    main_camera_frame = np.asanyarray(main_camera_frame.get_data())

    # Get Frames from Side camera
    side_camera_frame = side_cam_pipeline.wait_for_frames()
    side_camera_frame = side_camera_frame.get_color_frame()
    side_camera_frame = np.asanyarray(side_camera_frame.get_data())

    # Display the images
    cv2.imshow('Main_Camera', main_camera_frame)
    cv2.imshow('Side_Camera', side_camera_frame)

    # Return the Image frames
    return main_camera_frame, side_camera_frame