# Import Necessary Libraries
import pyrealsense2 as rs
import numpy as np
import sys
import os


# Get the Device connected
devices = rs.context().devices

# Get the Input Camera name and Create directory
camera = sys.argv[1]
os.makedirs(camera, exist_ok = True)

# For every Device connected
for device in devices:
            
    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the Camera streams
    camera_serial_number = device.get_info(rs.camera_info.serial_number)
    config.enable_device(camera_serial_number)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg = pipeline.start(config)

    # Get the Intrinsic parameters
    intrinsics = cfg.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsic_matrix =  [
                            [intrinsics.fx, 0, intrinsics.ppx],
                            [0, intrinsics.fy, intrinsics.ppy],
                            [0, 0, 1]
                        ]
    # Get distortion coefficients
    distortion_coeff = intrinsics.coeffs

    # Save the Calibration parameters
    np.save(camera + "/calibration_matrix", np.array(intrinsic_matrix))
    np.save(camera + "/distortion_coefficients", np.array(distortion_coeff))

    # Write the Camera Serial number into Text file
    with open(camera + "/serial_number.txt", "w") as file:
        file.write(camera_serial_number)