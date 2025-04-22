"""
Stereo Camera Calibration and Rectification Script
------------------------------------------------
This script performs the following operations:
1. Prompts the user for calibration board dimensions and input/output paths for left and right camera images.
2. Detects chessboard corners in paired left and right images and saves annotated images.
3. Logs image pairs that fail to detect chessboard corners in an error log.
4. Optionally performs stereo camera calibration using cv.stereoCalibrate() and rectification.
5. Generates sample rectified images for visualization.
6. Saves calibration and rectification data in YAML format.
7. Terminates when all processing is complete.

Based on OpenCV stereo calibration documentation as of 2025:
https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5
"""

import numpy as np
import cv2 as cv
import glob
import os
import uuid

def main():
    # --- Prompt for Calibration Board Information ---
    print("-+-+-+-+- Input Calibration Board Information -+-+-+-+-")
    inner_rows = int(input("Number of INNER rows: "))
    inner_cols = int(input("Number of INNER columns: "))
    board_size = (inner_rows, inner_cols)
    
    # --- Prompt for Only Generating Chessboard Corners ---
    print("\n-+-+-+-+- Only Generate Chessboard Corner Images? -+-+-+-+-")
    print('Answer "y" if you have not yet filtered out images that create poor chessboard corner estimation\nAnswer "n" if you already have removed poor quality images and want to find and store calibration and rectification data')
    only_corners_input = input("Answer (y/n): ").strip().lower()
    only_chessboard = only_corners_input.startswith('y')
    
    # --- Prompt for Input and Output Paths ---
    print("\n-+-+-+-+- Choose Input and Output Paths -+-+-+-+-")
    left_input_path = input("Left camera input path: ").strip()
    right_input_path = input("Right camera input path: ").strip()
    output_path = input("Output path: ").strip()
    file_suffix = input("Output filename suffix: ").strip()
    
    # Create output subdirectories
    chessboard_dir = os.path.join(output_path, "chessboardCorners")
    calib_data_dir = os.path.join(output_path, "calibrationData")
    rect_sample_dir = os.path.join(output_path, "rectificationSample")
    error_log_dir = os.path.join(output_path, "errorLogs")
    
    os.makedirs(chessboard_dir, exist_ok=True)
    os.makedirs(calib_data_dir, exist_ok=True)
    os.makedirs(rect_sample_dir, exist_ok=True)
    os.makedirs(error_log_dir, exist_ok=True)
    
    # --- Read Paired Images ---
    left_images = sorted(glob.glob(os.path.join(left_input_path, "*.png")))
    right_images = sorted(glob.glob(os.path.join(right_input_path, "*.png")))
    
    if len(left_images) != len(right_images):
        print("Mismatch in number of left and right images. Exiting.")
        return
    if not left_images:
        print("No PNG images found. Exiting.")
        return
    
    # --- Prepare for Chessboard Corner Detection ---
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((inner_rows * inner_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:inner subcontractors_cols].T.reshape(-1, 2)
    
    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left image plane
    imgpoints_right = []  # 2D points in right image plane
    no_corners_list = []  # List to store file names for image pairs without detected corners
    
    print("\n== Starting Chessboard Pattern Detection ==")
    for left_path, right_path in zip(left_images, right_images):
        left_img = cv.imread(left_path)
        right_img = cv.imread(right_path)
        left_gray = cv.cvtColor(left_img, cv.COLOR_BGR2GRAY)
        right_gray = cv.cvtColor(right_img, cv.COLOR_BGR2GRAY)
        
        # Find chessboard corners in both images
        ret_left, corners_left = cv.findChessboardCorners(left_gray, board_size, None)
        ret_right, corners_right = cv.findChessboardCorners(right_gray, board_size, None)
        
        if ret_left and ret_right:
            objpoints.append(objp)
            # Refine corner locations
            corners_left_refined = cv.cornerSubPix(left_gray, corners_left, (11, 11), (-1, -1), criteria)
            corners_right_refined = cv.cornerSubPix(right_gray, corners_right, (11, 11), (-1, -1), criteria)
            imgpoints_left.append(corners_left_refined)
            imgpoints_right.append(corners_right_refined)
            
            # Draw and save corners
            cv.drawChessboardCorners(left_img, board_size, corners_left_refined, ret_left)
            cv.drawChessboardCorners(right_img, board_size, corners_right_refined, ret_right)
            left_filename = os.path.basename(left_path)
            right_filename = os.path.basename(right_path)
            left_save_path = os.path.join(chessboard_dir, f"left_{left_filename}")
            right_save_path = os.path.join(chessboard_dir, f"right_{right_filename}")
            cv.imwrite(left_save_path, left_img)
            cv.imwrite(right_save_path, right_img)
            
            # Display images
            cv.imshow('Left Chessboard Corners', left_img)
            cv.imshow('Right Chessboard Corners', right_img)
            cv.waitKey(50)
        else:
            no_corners_list.append((os.path.basename(left_path), os.path.basename(right_path)))
        cv.destroyAllWindows()
    
    # Write error log
    if no_corners_list:
        error_log_file = os.path.join(error_log_dir, "no_chessboard_corners.txt")
        with open(error_log_file, "w") as f:
            for left_name, right_name in no_corners_list:
                f.write(f"Left: {left_name}, Right: {right_name}\n")
        print(f"Chessboard corners could not be found for {len(no_corners_list)} image pairs.")
        print(f"Log of problematic image pairs can be found in:\n {error_log_file}")
    
    print("== Completed Chessboard Pattern Detection ==")
    
    if only_chessboard:
        print("\nOnly chessboard corner images were generated. Exiting script.")
        return
    
    # --- Individual Camera Calibration ---
    print("\n== Starting Individual Camera Calibration ==")
    sample_left_img = cv.imread(left_images[0])
    left_gray_sample = cv.cvtColor(sample_left_img, cv.COLOR_BGR2GRAY)
    img_size = left_gray_sample.shape[::-1]
    
    # Left camera
    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv.calibrateCamera(
        objpoints, imgpoints_left, img_size, None, None
    )
    # Right camera
    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv.calibrateCamera(
        objpoints, imgpoints_right, img_size, None, None
    )
    
    # Save individual calibration data
    left_calib_yaml = os.path.join(calib_data_dir, f"left_calibration_{file_suffix}.yaml")
    fs = cv.FileStorage(left_calib_yaml, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrix", mtx_left)
    fs.write("distCoeffs", dist_left)
    fs.release()
    
    right_calib_yaml = os.path.join(calib_data_dir, f"right_calibration_{file_suffix}.yaml")
    fs = cv.FileStorage(right_calib_yaml, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrix", mtx_right)
    fs.write("distCoeffs", dist_right)
    fs.release()
    
    print("Individual camera calibration data saved to:")
    print(left_calib_yaml)
    print(right_calib_yaml)
    
    # --- Stereo Calibration ---
    print("\n== Starting Stereo Calibration ==")
    flags = (
        cv.CALIB_FIX_INTRINSIC +
        cv.CALIB_ZERO_TANGENT_DIST +
        cv.CALIB_SAME_FOCAL_LENGTH
    )
    stereocalib_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    
    ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx_left, dist_left, mtx_right, dist_right,
        img_size, criteria=stereocalib_criteria, flags=flags
    )
    
    # Save stereo calibration data
    stereo_calib_yaml = os.path.join(calib_data_dir, f"stereo_calibration_{file_suffix}.yaml")
    fs = cv.FileStorage(stereo_calib_yaml, cv.FILE_STORAGE_WRITE)
    fs.write("cameraMatrixLeft", mtx_left)
    fs.write("distCoeffsLeft", dist_left)
    fs.write("cameraMatrixRight", mtx_right)
    fs.write("distCoeffsRight", dist_right)
    fs.write("R", R)
    fs.write("T", T)
    fs.write("E", E)
    fs.write("F", F)
    fs.release()
    print("Stereo calibration data saved to:\n", stereo_calib_yaml)
    
    # --- Stereo Rectification ---
    print("\n== Performing Stereo Rectification ==")
    R1, R2, P1, P2, Q, roi_left, roi_right = cv.stereoRectify(
        mtx_left, dist_left, mtx_right, dist_right, img_size, R, T,
        flags=cv.CALIB_ZERO_DISPARITY, alpha=1
    )
    
    # Compute rectification maps
    left_mapx, left_mapy = cv.initUndistortRectifyMap(
        mtx_left, dist_left, R1, P1, img_size, cv.CV_32FC1
    )
    right_mapx, right_mapy = cv.initUndistortRectifyMap(
        mtx_right, dist_right, R2, P2, img_size, cv.CV_32FC1
    )
    
    # Save rectification data
    rect_yaml = os.path.join(calib_data_dir, f"rectification_data_{file_suffix}.yaml")
    fs = cv.FileStorage(rect_yaml, cv.FILE_STORAGE_WRITE)
    fs.write("R1", R1)
    fs.write("R2", R2)
    fs.write("P1", P1)
    fs.write("P2", P2)
    fs.write("Q", Q)
    fs.write("leftMapX", left_mapx)
    fs.write("leftMapY", left_mapy)
    fs.write("rightMapX", right_mapx)
    fs.write("rightMapY", right_mapy)
    fs.release()
    print("Rectification data saved to:\n", rect_yaml)
    
    # --- Generate Rectified Sample Images ---
    left_sample = cv.imread(left_images[0])
    right_sample = cv.imread(right_images[0])
    left_rectified = cv.remap(left_sample, left_mapx, left_mapy, cv.INTER_LINEAR)
    right_rectified = cv.remap(right_sample, right_mapx, right_mapy, cv.INTER_LINEAR)
    
    left_sample_path = os.path.join(rect_sample_dir, f"left_rectified_sample_{file_suffix}.png")
    right_sample_path = os.path.join(rect_sample_dir, f"right_rectified_sample_{file_suffix}.png")
    cv.imwrite(left_sample_path, left_rectified)
    cv.imwrite(right_sample_path, right_rectified)
    
    cv.imshow('Left Rectified Sample', left_rectified)
    cv.imshow('Right Rectified Sample', right_rectified)
    cv.waitKey(0)
    cv.destroyAllWindows()
    
    print("Rectified sample images saved to:")
    print(left_sample_path)
    print(right_sample_path)
    
    print("\n== Stereo Calibration and Rectification Complete ==")
    print("\nAll processing complete. Terminating script.")

if __name__ == "__main__":
    main()