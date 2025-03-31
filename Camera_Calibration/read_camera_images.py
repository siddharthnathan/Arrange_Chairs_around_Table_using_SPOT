# Import Necessary Libraries
import shutil
import sys
sys.path.append('..')
import time
import cv2
import os

# Import Necessary Scripts
import read_video_stream


# Configure and Stream Realsense Pipeline
main_camera_pipeline = read_video_stream.configure_and_stream_pipeline()

# Delete and Create Directory to store images
if os.path.exists('checkerboard_images'):
    shutil.rmtree('checkerboard_images')
os.mkdir('checkerboard_images')

# Read Image frames from both the Cameras
try:

    # Initialise Number of Images count
    num_images = 0

    # Read Image frames continuously
    while True:
        
        # Read Image frames from Pipelines
        main_camera_frame = read_video_stream.read_frames_from_pipelines(main_camera_pipeline)

        # Display the Image from Main Camera
        cv2.imshow('Main_Camera', main_camera_frame)
        cv2.waitKey(1)
        time.sleep(0.5)

        # Save Images into Directory
        cv2.imwrite('checkerboard_images/image_' + str(num_images) + '.jpg', main_camera_frame)
        num_images = num_images + 1

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_camera_pipeline.stop()