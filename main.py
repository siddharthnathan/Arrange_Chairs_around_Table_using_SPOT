# Import Necessary Scripts
import read_video_stream
import aruco_tag_detection

# Import Necessary Libraries
import numpy as np
import cv2


# Configure and Stream Realsense Pipeline
main_cam_pipeline, side_cam_pipeline = read_video_stream.configure_and_stream_pipeline()

# Read Image frames from both the Cameras
try:

    # Read Image frames continuously
    while True:
        
        # Read Image frames from Pipelines
        main_camera_frame, side_camera_frame = read_video_stream.read_frames_from_pipelines(main_cam_pipeline, side_cam_pipeline)

        aruco_tags_detected_main_cam = aruco_tag_detection.detect_aruco_markers_in_frame(main_camera_frame, 'Main_Camera', aruco_type = cv2.aruco.DICT_APRILTAG_36h11)
        print(aruco_tags_detected_main_cam)

        # Quit when Q key is Pressed
        if cv2.waitKey(1) == ord('q'):
            break

# Stop streaming finally
finally:
    main_cam_pipeline.stop()
    side_cam_pipeline.stop()