import pyrealsense2 as rs
import numpy as np

'''
Intrincs and Exrinsic information available from command line with RealSense SDK
$ rs-enumerate-devices -c
'''
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
    intrinsics = cfg.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsic_matrix = [
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ]
    # get distortion coefficients
    distortion_coeff = intrinsics.coeffs

    # Save the Coefficients
    np.save("calibration_matrix", np.array(intrinsic_matrix))
    np.save("distortion_coefficients", np.array(distortion_coeff))
