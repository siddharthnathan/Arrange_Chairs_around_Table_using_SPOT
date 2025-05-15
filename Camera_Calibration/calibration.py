# Import Necessary Libraries
import pyrealsense2 as rs
import numpy as np


# Get the Devices connected
devices = rs.context().devices

# For every Device connected
for device in devices:
            
    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the Camera streams
    config.enable_device(device.get_info(rs.camera_info.serial_number))
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

    # Save the Coefficients
    np.save("Main_Camera/calibration_matrix", np.array(intrinsic_matrix))
    np.save("Main_Camera/distortion_coefficients", np.array(distortion_coeff))
