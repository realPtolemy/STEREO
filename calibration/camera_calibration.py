"""
Camera Calibration and Undistortion Script
------------------------------------------
This script performs the following operations:
1. Prompts the user for calibration board dimensions and input/output paths.
2. Detects chessboard corners in the provided images and saves annotated images.
3. Logs images that fail to detect chessboard corners in an error log.
4. Optionally performs camera calibration and computes undistortion maps.
5. Generates a sample undistorted image.
6. COMMENTED OUT - OPTIONAL: Loops through all input images, displaying their undistorted versions (not saved).
7. Terminates when all processing is complete.

This code is based on the OpenCV calibration tutorial as per 2025-MAR-12:
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""

import numpy as np
import cv2 as cv
import glob
import os

def main():
    # --- Prompt for Calibration Board Information ---
    print("-+-+-+-+- Input Calibration Board Information -+-+-+-+-")
    inner_rows = int(input("Number of INNER rows: "))
    inner_cols = int(input("Number of INNER columns: "))
    board_size = (inner_rows, inner_cols)
    
    # --- Prompt for Only Generating Chessboard Corners ---
    print("\n-+-+-+-+- Only Generate Chessboard Corner Images? -+-+-+-+-")
    print('Answer "y" if you have not yet filtered out images that create poor chessboard corner estimation\nAnswer "n" if you already have removed poor quality images and want to find and store calibration and distortion data')
    only_corners_input = input("Answer (y/n): ").strip().lower()
    only_chessboard = only_corners_input.startswith('y')
    
    # --- Prompt for Input and Output Paths ---
    print("\n-+-+-+-+- Choose Input and Output Paths -+-+-+-+-")
    input_path = input("Input path: ").strip()
    output_path = input("Output path: ").strip()
    file_suffix = input("Output filename suffix: ").strip()
    
    # Create output subdirectories if they do not exist
    chessboard_dir = os.path.join(output_path, "chessboardCorners")
    calib_data_dir = os.path.join(output_path, "calibrationData")
    undist_sample_dir = os.path.join(output_path, "undistortionSample")
    error_log_dir = os.path.join(output_path, "errorLogs")
    
    os.makedirs(chessboard_dir, exist_ok=True)
    os.makedirs(calib_data_dir, exist_ok=True)
    os.makedirs(undist_sample_dir, exist_ok=True)
    os.makedirs(error_log_dir, exist_ok=True)
    
    # --- Read Images ---
    # Assume PNG images; add more extensions if needed..!
    image_paths = glob.glob(os.path.join(input_path, "*.png"))
    if not image_paths:
        print("No PNG images found. Exiting.")
        return

    # --- Prepare for Chessboard Corner Detection ---
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Prepare object points which will reflect ground truth for the chessboard’s geometry.
    # e.g. (0,0,0), (1,0,0), …,(inner_rows-1, inner_cols-1, 0)
    objp = np.zeros((inner_rows * inner_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:inner_rows, 0:inner_cols].T.reshape(-1, 2)
    
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    no_corners_list = []  # List to store file names for images without detected corners
    
    print("\n== Starting Chessboard Pattern Detection ==")
    for img_path in image_paths:
        img = cv.imread(img_path)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chessboard corners
        ret, corners = cv.findChessboardCorners(gray, board_size, None)
        
        if ret:
            objpoints.append(objp)
            # Refine corner locations
            corners_refined = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners_refined)
            
            # Draw and save the corners on the image
            cv.drawChessboardCorners(img, board_size, corners_refined, ret)
            filename = os.path.basename(img_path)
            save_path = os.path.join(chessboard_dir, filename)
            cv.imwrite(save_path, img)
            cv.imshow('Chessboard Corners', img)
            cv.waitKey(50)  # Show image for 50 ms. Too slow? Good to wait, so set to 1 ms, or comment out imshow & wait
        else:
            # Log file name of images with no detected corners
            no_corners_list.append(os.path.basename(img_path))
        cv.destroyAllWindows()

     # Write error log if any images did not have chessboard corners detected
    if no_corners_list:
        error_log_file = os.path.join(error_log_dir, "no_chessboard_corners.txt")
        with open(error_log_file, "w") as f:
            for name in no_corners_list:
                f.write(name + "\n")
        print(f"Chessboard corners could not be found for {len(no_corners_list)} images.")
        print(f"Log of problematic images can be found in:\n {error_log_file}")
    
    print("== Completed Chessboard Pattern Detection ==")
    
    # If the user only wants to generate chessboard corner images, exit here.
    if only_chessboard:
        print("\nOnly chessboard corner images were generated. Exiting script.")
        return
    
    # --- Camera Calibration ---
    print("\n== Starting Camera Calibration ... this may take some time ==")
    # Use the shape of the first image as a reference for calibration
    sample_img = cv.imread(image_paths[0])
    gray_sample = cv.cvtColor(sample_img, cv.COLOR_BGR2GRAY)
    
    # The good 'ol linear algebra stuff
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray_sample.shape[::-1], None, None)
    # TYPE         PARAMETER    DESCRIPTION
    # ------------------------------------------------------------------------------------------------------------
    # Intrinsic    ret          Reprojection Error (Quality metric of the calibration)
    # Intrinsic    mtx          Camera Matrix (3x3 Matrix with focal lengths, optical center and skew coefficient)
    # Intrinsic    dist         Distortion Coefficent (Lens distortion)
    # Extrinsic    rvecs        Rotation Vectors (Camera orientation, its rotation of (x,y,z)-axis)
    # Extrinsic    tvecs        Translation Vectors (Camera position)
    # ------------------------------------------------------------------------------------------------------------
    # The camera matrix describes how the camera projects 3D points from the world onto a 2D image plane

    # Save calibration data in YAML format
    calib_yaml_path = os.path.join(calib_data_dir, f"calibration_data_{file_suffix}.yaml")
    fs = cv.FileStorage(calib_yaml_path, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrix", mtx)
    fs.write("distCoeffs", dist)
    fs.startWriteStruct("rotationVectors", cv.FILE_NODE_SEQ)
    for rvec in rvecs:
        fs.write("", rvec)
    fs.endWriteStruct()
    fs.startWriteStruct("translationVectors", cv.FILE_NODE_SEQ)
    for tvec in tvecs:
        fs.write("", tvec)
    fs.endWriteStruct()
    fs.release()
    print("Calibration data saved to:\n", calib_yaml_path)
    print("== Camera Calibration Completed ==")
    
    # --- Undistortion ---
    print("\n== Performing Undistortion ==")
    h, w = gray_sample.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_32FC1)
    
    # Generate one undistortion sample image (using the first image)
    undist_sample = cv.remap(sample_img, mapx, mapy, cv.INTER_LINEAR)
    sample_image_path = os.path.join(undist_sample_dir, f"undistortion_sample_{file_suffix}.png")
    cv.imwrite(sample_image_path, undist_sample)
    cv.imshow('Undistorted Sample', undist_sample)
    cv.waitKey(0)
    cv.destroyAllWindows()
    print("Undistortion sample image saved to:\n", sample_image_path)

    # Save undistortion maps and parameters
    undist_yaml_path = os.path.join(calib_data_dir, f"undistortion_data_{file_suffix}.yaml")
    fs = cv.FileStorage(undist_yaml_path, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrix", mtx)
    fs.write("distCoeffs", dist)
    fs.write("undistortMap_x", mapx)
    fs.write("undistortMap_y", mapy)
    fs.release()
    print("Undistortion data saved to:\n", undist_yaml_path)

    ## --- Loop through and display all undistorted images --- 
    ## OPTIONAL
    #print("\n== Displaying All Undistorted Images ==")
    #for img_path in image_paths:
    #    img = cv.imread(img_path)
    #    undistorted_img = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
    #    cv.imshow("Undistorted Image", undistorted_img)
    #    key = cv.waitKey(0)  # Wait indefinitely until a key is pressed
    #    # Exit early if the 'Esc' key is pressed (key code 27)
    #    if key == 27:
    #        break
    #cv.destroyAllWindows()
    
    print("\n== Undistortion Complete ==")
    print("\nAll processing complete. Terminating script.")

if __name__ == "__main__":
    main()