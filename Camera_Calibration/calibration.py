# Import Necessary Libraries
import numpy as np
import argparse
import cv2
import os


# Define a Function to Calibrate camera using its Checkerboard images
def calibrate(dirpath, square_size, width, height, visualize = False):

    # Define the Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ...., (8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp = objp * square_size

    # Initialise arrays to store object points and image points from all the images
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    # For every Image in Checkerboard Directory
    for fname in os.listdir(dirpath):

        # Read the Image and Convert into Grayscale
        img = cv2.imread(os.path.join(dirpath, fname))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the Chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
 
        # If found
        if ret:

            # Add Object points and Image points
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and Display the Corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

        # Visualise Checkerboard image with corners
        if visualize:
            cv2.imshow('img', img)
            cv2.waitKey(0)

    # Calibrate Camera using the Object points and Image points
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Return the Calibration parameters
    return [ret, mtx, dist, rvecs, tvecs]



# Define the Main function
if __name__ == '__main__':

    # Define the Argument parser and its arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--dir", required = True, help = "Path to folder containing checkerboard images for calibration")
    ap.add_argument("-w", "--width", type = int, help = "Width of checkerboard (default = 2)",  default = 5)
    ap.add_argument("-t", "--height", type= int, help = "Height of checkerboard (default = 1)", default = 3)
    ap.add_argument("-s", "--square_size", type = float, default = 0.043, help = "Length of one edge (in metres)")
    ap.add_argument("-v", "--visualize", type = str, default = "False", help = "To visualize each checkerboard image")
    
    # Retrieve all the Arguments parameters
    args = vars(ap.parse_args())
    dirpath = args['dir']
    square_size = args['square_size']
    width = args['width']
    height = args['height']

    # Retrieve the Visualization flag parameter
    if args["visualize"].lower() == "true":
        visualize = True
    else:
        visualize = False

    # Calibrate the Camera to get its Matrix coefficients and Distortion coefficients
    _, mtx, dist, _, _ = calibrate(dirpath, square_size, width = width, height = height, visualize = visualize)

    # Display the Coefficients
    print("\nCamera matrix Coefficients:")
    print(mtx)
    print("\nCamera distortion Coefficients:")
    print(dist)

    # Save the Coefficients
    np.save("calibration_matrix", mtx)
    np.save("distortion_coefficients", dist)
