"""
Stereo Camera Calibration and Rectification Script
------------------------------------------------
This script is tailored for a dog-mounted stereo camera setup with a 5–10 cm baseline, nearly parallel cameras, and same height. It handles image quality differences (clutter, lighting variations) and prioritizes close-range images (< 0.5 m). It performs:
1. Prompts for calibration board dimensions, square size, input/output paths, and mono calibration YAML files.
2. Detects chessboard corners with robust settings and preprocessing to minimize clutter/lighting effects.
3. Estimates chessboard distance and filters distant images (> 0.5 m).
4. Logs image pairs that fail corner detection or have high reprojection errors.
5. Loads intrinsic parameters from mono calibration YAML files.
6. Filters image pairs based on per-image reprojection errors (tight threshold).
7. Performs stereo calibration with fixed intrinsics, optimized for small baseline.
8. Generates rectified images with epipolar lines and reprojection visualization for debugging.
9. Saves calibration, rectification, and reprojection data in YAML/text/image formats.
10. Terminates when complete.

Based on OpenCV stereo calibration documentation as of 2025:
https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5
"""

import numpy as np
import cv2 as cv
import glob
import os
import time

def compute_per_image_reproj_error(objpoints, imgpoints_left, imgpoints_right, mtx_left, dist_left, mtx_right, dist_right, R, T):
    """Compute per-image reprojection errors for stereo calibration."""
    errors = []
    for i, (objp, imgl, imgr) in enumerate(zip(objpoints, imgpoints_left, imgpoints_right)):
        rvec, tvec = cv.solvePnP(objp, imgl, mtx_left, dist_left)[1:]
        proj_left = cv.projectPoints(objp, rvec, tvec, mtx_left, dist_left)[0].reshape(-1, 2)
        error_left = np.mean(np.sqrt(np.sum((imgl.reshape(-1, 2) - proj_left) ** 2, axis=1)))
        
        rvec, tvec = cv.solvePnP(objp, imgr, mtx_right, dist_right)[1:]
        proj_right = cv.projectPoints(objp, rvec, tvec, mtx_right, dist_right)[0].reshape(-1, 2)
        error_right = np.mean(np.sqrt(np.sum((imgr.reshape(-1, 2) - proj_right) ** 2, axis=1)))
        
        errors.append((error_left, error_right))
    return errors

def estimate_chessboard_distance(objp, imgpoints, mtx, dist):
    """Estimate the z-distance (in meters) of the chessboard from the camera."""
    rvec, tvec = cv.solvePnP(objp, imgpoints, mtx, dist)[1:]
    z_distance = tvec[2][0] / 1000.0  # Convert mm to meters
    return z_distance

def visualize_reprojection(img, imgpoints, proj_points, filename, output_dir):
    """Visualize detected (green) and projected (red) points with connecting lines (blue)."""
    img_copy = img.copy()
    for det, proj in zip(imgpoints.reshape(-1, 2), proj_points.reshape(-1, 2)):
        det = tuple(map(int, det))
        proj = tuple(map(int, proj))
        cv.circle(img_copy, det, 5, (0, 255, 0), 1)
        cv.circle(img_copy, proj, 5, (0, 0, 255), 1)
        cv.line(img_copy, det, proj, (255, 0, 0), 1)
    cv.imwrite(os.path.join(output_dir, filename), img_copy)
    return img_copy

def main():
    print("-+-+-+-+- Input Calibration Board Information -+-+-+-+-")
    inner_rows = int(input("Number of INNER rows: "))
    inner_cols = int(input("Number of INNER columns: "))
    square_size = float(input("Chessboard square size (in millimeters): "))
    board_size = (inner_rows, inner_cols)
    
    print("\n-+-+-+-+- Only Generate Chessboard Corner Images? -+-+-+-+-")
    print('Answer "y" if you have not yet filtered out images that create poor chessboard corner estimation\nAnswer "n" if you already have removed poor quality images and want to find and store calibration and rectification data')
    only_corners_input = input("Answer (y/n): ").strip().lower()
    only_chessboard = only_corners_input.startswith('y')
    
    print("\n-+-+-+-+- Choose Input and Output Paths -+-+-+-+-")
    left_input_path = input("Left camera input path: ").strip()
    right_input_path = input("Right camera input path: ").strip()
    output_path = input("Output path: ").strip()
    file_suffix = input("Output filename suffix (e.g., '1', 'test', 'calib_v1', without extension): ").strip()
    
    print("\n-+-+-+-+- Choose Mono Calibration Files -+-+-+-+-")
    left_calib_yaml = input("Path to left camera mono calibration YAML: ").strip()
    right_calib_yaml = input("Path to right camera mono calibration YAML: ").strip()
    
    chessboard_dir = os.path.join(output_path, "chessboardCorners")
    calib_data_dir = os.path.join(output_path, "calibrationData")
    rect_sample_dir = os.path.join(output_path, "rectificationSample")
    reproj_sample_dir = os.path.join(output_path, "reprojectionSample")
    error_log_dir = os.path.join(output_path, "errorLogs")
    
    os.makedirs(chessboard_dir, exist_ok=True)
    os.makedirs(calib_data_dir, exist_ok=True)
    os.makedirs(rect_sample_dir, exist_ok=True)
    os.makedirs(reproj_sample_dir, exist_ok=True)
    os.makedirs(error_log_dir, exist_ok=True)
    
    print("\n== Loading Mono Calibration Parameters ==")
    try:
        fs = cv.FileStorage(left_calib_yaml, cv.FILE_STORAGE_READ)
        mtx_left = fs.getNode("cameraMatrix").mat()
        dist_left = fs.getNode("distCoeffs").mat()
        fs.release()
        print("Left camera parameters loaded successfully.")
    except Exception as e:
        print(f"Error loading left calibration file: {str(e)}")
        return
    
    try:
        fs = cv.FileStorage(right_calib_yaml, cv.FILE_STORAGE_READ)
        mtx_right = fs.getNode("cameraMatrix").mat()
        dist_right = fs.getNode("distCoeffs").mat()
        fs.release()
        print("Right camera parameters loaded successfully.")
    except Exception as e:
        print(f"Error loading right calibration file: {str(e)}")
        return
    
    print(f"Left camera matrix:\n{mtx_left}")
    print(f"Left distortion coefficients: {dist_left.flatten()}")
    print(f"Right camera matrix:\n{mtx_right}")
    print(f"Right distortion coefficients: {dist_right.flatten()}")
    
    left_images = sorted(glob.glob(os.path.join(left_input_path, "*.png")))
    right_images = sorted(glob.glob(os.path.join(right_input_path, "*.png")))
    
    if len(left_images) != len(right_images):
        print("Mismatch in number of left and right images. Exiting.")
        return
    if not left_images:
        print("No PNG images found. Exiting.")
        return
    
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    objp = np.zeros((inner_rows * inner_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2) * square_size
    
    objpoints = []
    imgpoints_left = []
    imgpoints_right = []
    no_corners_list = []
    image_pairs = []
    
    print("\n== Starting Chessboard Pattern Detection ==")
    for i, (left_path, right_path) in enumerate(zip(left_images, right_images)):
        start_time = time.time()
        print(f"Processing image pair {i+1}/{len(left_images)}: {os.path.basename(left_path)} | {os.path.basename(right_path)}")
        
        left_img = cv.imread(left_path)
        right_img = cv.imread(right_path)
        if left_img is None or right_img is None:
            print(f"  Failed to load images: Left={left_img is None}, Right={right_img is None}")
            no_corners_list.append((os.path.basename(left_path), os.path.basename(right_path)))
            continue
        
        print("  Converting to grayscale and preprocessing...")
        left_gray = cv.cvtColor(left_img, cv.COLOR_BGR2GRAY)
        right_gray = cv.cvtColor(right_img, cv.COLOR_BGR2GRAY)
        left_gray = cv.GaussianBlur(left_gray, (3, 3), 0)
        right_gray = cv.GaussianBlur(right_gray, (3, 3), 0)
        left_gray = cv.equalizeHist(left_gray)
        right_gray = cv.equalizeHist(right_gray)
        
        print("  Detecting chessboard corners...")
        ret_left, corners_left = cv.findChessboardCorners(
            left_gray, board_size, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK
        )
        ret_right, corners_right = cv.findChessboardCorners(
            right_gray, board_size, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK
        )
        
        if ret_left and ret_right:
            print("  Corners detected in both images.")
            print("  Refining corners...")
            corners_left_refined = cv.cornerSubPix(left_gray, corners_left, (5, 5), (-1, -1), criteria)
            corners_right_refined = cv.cornerSubPix(right_gray, corners_right, (5, 5), (-1, -1), criteria)
            
            # Estimate chessboard distance
            z_distance_left = estimate_chessboard_distance(objp, corners_left_refined, mtx_left, dist_left)
            z_distance_right = estimate_chessboard_distance(objp, corners_right_refined, mtx_right, dist_right)
            max_distance = 0.5  # 0.5 meters
            if z_distance_left > max_distance or z_distance_right > max_distance:
                print(f"  Skipping pair: Chessboard too far (Left: {z_distance_left:.2f} m, Right: {z_distance_right:.2f} m)")
                no_corners_list.append((os.path.basename(left_path), os.path.basename(right_path)))
                continue
            
            objpoints.append(objp)
            imgpoints_left.append(corners_left_refined)
            imgpoints_right.append(corners_right_refined)
            
            print("  Drawing and saving corners...")
            cv.drawChessboardCorners(left_img, board_size, corners_left_refined, ret_left)
            cv.drawChessboardCorners(right_img, board_size, corners_right_refined, ret_right)
            left_filename = os.path.basename(left_path)
            right_filename = os.path.basename(right_path)
            left_save_path = os.path.join(chessboard_dir, f"left_{left_filename}")
            right_save_path = os.path.join(chessboard_dir, f"right_{right_filename}")
            cv.imwrite(left_save_path, left_img)
            cv.imwrite(right_save_path, right_img)
            image_pairs.append((left_filename, right_filename, left_img, right_img))
        else:
            print(f"  Corners not detected: Left={ret_left}, Right={ret_right}")
            no_corners_list.append((os.path.basename(left_path), os.path.basename(right_path)))
        
        elapsed_time = time.time() - start_time
        print(f"  Completed pair {i+1} in {elapsed_time:.2f} seconds")

    cv.destroyAllWindows()
    
    if no_corners_list:
        error_log_file = os.path.join(error_log_dir, "no_chessboard_corners.txt")
        with open(error_log_file, "w") as f:
            for left_name, right_name in no_corners_list:
                f.write(f"Left: {left_name}, Right: {right_name}\n")
        print(f"Chessboard corners could not be found or too far for {len(no_corners_list)} image pairs.")
        print(f"Log of problematic image pairs can be found in:\n {error_log_file}")
    
    print("== Completed Chessboard Pattern Detection ==")
    
    if only_chessboard:
        print("\nOnly chessboard corner images were generated. Exiting script.")
        return
    
    print("\n== Image Pair Selection ==")
    print(f"  Detected {len(objpoints)} valid image pairs. Using all pairs for calibration.")
    
    print("\n== Validating Input Data for Stereo Calibration ==")
    print(f"    Number of object points: {len(objpoints)}")
    print(f"    Number of left image points: {len(imgpoints_left)}")
    print(f"    Number of right image points: {len(imgpoints_right)}")
    if not (len(objpoints) == len(imgpoints_left) == len(imgpoints_right)):
        print("    Error: Mismatch in number of point sets. Exiting.")
        return
    if len(objpoints) < 3:
        print("    Error: Too few valid image pairs (minimum 3 required). Exiting.")
        return
    
    print(f"    Object points shape (first set): {objpoints[0].shape}")
    print(f"    Left image points shape (first set): {imgpoints_left[0].shape}")
    print(f"    Right image points shape (first set): {imgpoints_right[0].shape}")
    sample_left_img = cv.imread(left_images[0])
    left_gray_sample = cv.cvtColor(sample_left_img, cv.COLOR_BGR2GRAY)
    img_size = left_gray_sample.shape[::-1]
    print(f"    Image size: {img_size}")
    
    for i, (objp, imgl, imgr) in enumerate(zip(objpoints, imgpoints_left, imgpoints_right)):
        if np.any(np.isnan(objp)) or np.any(np.isinf(objp)):
            print(f"    Error: NaN or Inf in objpoints at index {i}")
            return
        if np.any(np.isnan(imgl)) or np.any(np.isinf(imgl)):
            print(f"    Error: NaN or Inf in imgpoints_left at index {i}")
            return
        if np.any(np.isnan(imgr)) or np.any(np.isinf(imgr)):
            print(f"    Error: NaN or Inf in imgpoints_right at index {i}")
            return
    
    print("\n== Starting Stereo Calibration ==")
    flags = (
        cv.CALIB_FIX_INTRINSIC +
        cv.CALIB_USE_INTRINSIC_GUESS
    )
    stereocalib_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 1e-3)
    
    print("  Executing cv.stereoCalibrate...")
    start_time = time.time()
    try:
        ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
            objpoints, imgpoints_left, imgpoints_right,
            mtx_left, dist_left, mtx_right, dist_right,
            img_size, criteria=stereocalib_criteria, flags=flags
        )
        elapsed_time = time.time() - start_time
        print(f"  cv.stereoCalibrate completed in {elapsed_time:.2f} seconds")
        print(f"  Stereo reprojection error: {ret}")
        print(f"  Rotation matrix R:\n{R}")
        print(f"  Translation vector T:\n{T}")
        if ret > 1.0:
            print("  Warning: High reprojection error. Calibration may be unreliable.")
    except cv.error as e:
        elapsed_time = time.time() - start_time
        print(f"  Error in cv.stereoCalibrate after {elapsed_time:.2f} seconds: {str(e)}")
        return
    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"  Unexpected error in cv.stereoCalibrate after {elapsed_time:.2f} seconds: {str(e)}")
        return
    
    print("  Computing per-image reprojection errors and visualizing...")
    per_image_errors = []
    for i, (objp, imgl, imgr, (left_name, right_name, left_img, right_img)) in enumerate(zip(objpoints, imgpoints_left, imgpoints_right, image_pairs)):
        rvec, tvec = cv.solvePnP(objp, imgl, mtx_left, dist_left)[1:]
        proj_left = cv.projectPoints(objp, rvec, tvec, mtx_left, dist_left)[0].reshape(-1, 2)
        error_left = np.mean(np.sqrt(np.sum((imgl.reshape(-1, 2) - proj_left) ** 2, axis=1)))
        left_reproj_img = visualize_reprojection(
            left_img, imgl, proj_left, f"left_reproj_{left_name}", reproj_sample_dir
        )
        
        rvec, tvec = cv.solvePnP(objp, imgr, mtx_right, dist_right)[1:]
        proj_right = cv.projectPoints(objp, rvec, tvec, mtx_right, dist_right)[0].reshape(-1, 2)
        error_right = np.mean(np.sqrt(np.sum((imgr.reshape(-1, 2) - proj_right) ** 2, axis=1)))
        right_reproj_img = visualize_reprojection(
            right_img, imgr, proj_right, f"right_reproj_{right_name}", reproj_sample_dir
        )
        
        per_image_errors.append((error_left, error_right))
        
        if i == 0:
            cv.imshow('Left Reprojection Sample', left_reproj_img)
            cv.imshow('Right Reprojection Sample', right_reproj_img)
            cv.waitKey(0)
            cv.destroyAllWindows()
    
    error_log_file = os.path.join(error_log_dir, f"reprojection_errors_{file_suffix}.txt")
    with open(error_log_file, "w") as f:
        f.write("Image Pair | Left Error | Right Error\n")
        for i, (left_err, right_err) in enumerate(per_image_errors):
            f.write(f"{image_pairs[i][0]} | {image_pairs[i][1]} | {left_err:.4f} | {right_err:.4f}\n")
    print(f"Per-image reprojection errors saved to:\n {error_log_file}")
    
    error_threshold = 0.5
    valid_indices = [
        i for i, (left_err, right_err) in enumerate(per_image_errors)
        if left_err < error_threshold and right_err < error_threshold
    ]
    if len(valid_indices) < len(objpoints):
        print(f"  Filtering {len(objpoints) - len(valid_indices)} image pairs with high reprojection errors (> {error_threshold} pixels).")
        objpoints = [objpoints[i] for i in valid_indices]
        imgpoints_left = [imgpoints_left[i] for i in valid_indices]
        imgpoints_right = [imgpoints_right[i] for i in valid_indices]
        image_pairs = [image_pairs[i] for i in valid_indices]
        print(f"  Remaining valid image pairs: {len(objpoints)}")
    
    if len(objpoints) >= 3:
        print("  Rerunning cv.stereoCalibrate with filtered image pairs...")
        start_time = time.time()
        try:
            ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv.stereoCalibrate(
                objpoints, imgpoints_left, imgpoints_right,
                mtx_left, dist_left, mtx_right, dist_right,
                img_size, criteria=stereocalib_criteria, flags=flags
            )
            elapsed_time = time.time() - start_time
            print(f"  cv.stereoCalibrate (filtered) completed in {elapsed_time:.2f} seconds")
            print(f"  Stereo reprojection error (filtered): {ret}")
            print(f"  Rotation matrix R (filtered):\n{R}")
            print(f"  Translation vector T (filtered):\n{T}")
            if ret > 1.0:
                print("  Warning: High reprojection error after filtering. Calibration may be unreliable.")
        except cv.error as e:
            elapsed_time = time.time() - start_time
            print(f"  Error in cv.stereoCalibrate (filtered) after {elapsed_time:.2f} seconds: {str(e)}")
            return
        except Exception as e:
            elapsed_time = time.time() - start_time
            print(f"  Unexpected error in cv.stereoCalibrate (filtered) after {elapsed_time:.2f} seconds: {str(e)}")
            return
    else:
        print("  Error: Too few valid image pairs after filtering (< 3). Exiting.")
        return
    
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
    
    with open(os.path.join(calib_data_dir, f"stereo_params_{file_suffix}.txt"), "w") as f:
        f.write(f"Reprojection Error: {ret}\n")
        f.write(f"R:\n{R}\n")
        f.write(f"T:\n{T}\n")
    
    print("\n== Performing Stereo Rectification ==")
    start_time = time.time()
    try:
        R1, R2, P1, P2, Q, roi_left, roi_right = cv.stereoRectify(
            mtx_left, dist_left, mtx_right, dist_right, img_size, R, T,
            flags=cv.CALIB_ZERO_DISPARITY, alpha=0.5
        )
        elapsed_time = time.time() - start_time
        print(f"  cv.stereoRectify completed in {elapsed_time:.2f} seconds")
        print("  Rectification parameters:")
        print(f"    R1:\n{R1}")
        print(f"    R2:\n{R2}")
        print(f"    P1:\n{P1}")
        print(f"    P2:\n{P2}")
        print(f"    Q:\n{Q}")
        print(f"    Left ROI: {roi_left}")
        print(f"    Right ROI: {roi_right}")
        with open(os.path.join(calib_data_dir, f"stereo_params_{file_suffix}.txt"), "a") as f:
            f.write(f"P1:\n{P1}\n")
            f.write(f"P2:\n{P2}\n")
            f.write(f"Left ROI: {roi_left}\n")
            f.write(f"Right ROI: {roi_right}\n")
    except cv.error as e:
        elapsed_time = time.time() - start_time
        print(f"  Error in cv.stereoRectify after {elapsed_time:.2f} seconds: {str(e)}")
        return
    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"  Unexpected error in cv.stereoRectify after {elapsed_time:.2f} seconds: {str(e)}")
        return
    
    print("  Computing rectification maps...")
    start_time = time.time()
    try:
        left_mapx, left_mapy = cv.initUndistortRectifyMap(
            mtx_left, dist_left, R1, P1, img_size, cv.CV_32FC1
        )
        right_mapx, right_mapy = cv.initUndistortRectifyMap(
            mtx_right, dist_right, R2, P2, img_size, cv.CV_32FC1
        )
        elapsed_time = time.time() - start_time
        print(f"  Rectification maps computed in {elapsed_time:.2f} seconds")
        if np.any(np.isnan(left_mapx)) or np.any(np.isinf(left_mapx)):
            print("    Error: NaN or Inf in left_mapx")
            return
        if np.any(np.isnan(left_mapy)) or np.any(np.isinf(left_mapy)):
            print("    Error: NaN or Inf in left_mapy")
            return
        if np.any(np.isnan(right_mapx)) or np.any(np.isinf(right_mapx)):
            print("    Error: NaN or Inf in right_mapx")
            return
        if np.any(np.isnan(right_mapy)) or np.any(np.isinf(right_mapy)):
            print("    Error: NaN or Inf in right_mapy")
            return
        if left_mapx.min() < 0 or left_mapx.max() > img_size[0] or left_mapy.min() < 0 or left_mapy.max() > img_size[1]:
            print("Warning: Left rectification map out of image bounds")
        if right_mapx.min() < 0 or right_mapx.max() > img_size[0] or right_mapy.min() < 0 or right_mapy.max() > img_size[1]:
            print("Warning: Right rectification map out of image bounds")
        print(f"  Left mapx stats: min={left_mapx.min()}, max={left_mapx.max()}, mean={left_mapx.mean()}")
        print(f"  Left mapy stats: min={left_mapy.min()}, max={left_mapy.max()}, mean={left_mapy.mean()}")
        print(f"  Right mapx stats: min={right_mapx.min()}, max={right_mapx.max()}, mean={right_mapx.mean()}")
        print(f"  Right mapy stats: min={right_mapy.min()}, max={right_mapy.max()}, mean={right_mapy.mean()}")
    except cv.error as e:
        elapsed_time = time.time() - start_time
        print(f"  Error in initUndistortRectifyMap after {elapsed_time:.2f} seconds: {str(e)}")
        return
    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"  Unexpected error in initUndistortRectifyMap after {elapsed_time:.2f} seconds: {str(e)}")
        return
    
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
    
    print("  Generating rectified sample images...")
    left_sample = cv.imread(left_images[0])
    right_sample = cv.imread(right_images[0])
    if left_sample is None or right_sample is None:
        print("    Error: Failed to load sample images for rectification")
        return
    start_time = time.time()
    try:
        left_rectified = cv.remap(left_sample, left_mapx, left_mapy, cv.INTER_LINEAR)
        right_rectified = cv.remap(right_sample, right_mapx, right_mapy, cv.INTER_LINEAR)
        elapsed_time = time.time() - start_time
        print(f"  Rectified images generated in {elapsed_time:.2f} seconds")
        if np.all(left_rectified == left_rectified[0, 0]) or np.all(right_rectified == right_rectified[0, 0]):
            print("    Warning: Rectified images appear to be blank or uniform")
    except cv.error as e:
        elapsed_time = time.time() - start_time
        print(f"  Error in cv.remap after {elapsed_time:.2f} seconds: {str(e)}")
        return
    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"  Unexpected error in cv.remap after {elapsed_time:.2f} seconds: {str(e)}")
        return
    
    left_rectified_with_lines = left_rectified.copy()
    right_rectified_with_lines = right_rectified.copy()
    for y in range(0, left_rectified.shape[0], 50):
        cv.line(left_rectified_with_lines, (0, y), (left_rectified.shape[1], y), (0, 255, 0), 1)
        cv.line(right_rectified_with_lines, (0, y), (left_rectified.shape[1], y), (0, 255, 0), 1)
    
    left_sample_path = os.path.join(rect_sample_dir, f"left_rectified_sample_{file_suffix}.png")
    right_sample_path = os.path.join(rect_sample_dir, f"right_rectified_sample_{file_suffix}.png")
    left_lines_path = os.path.join(rect_sample_dir, f"left_rectified_lines_{file_suffix}.png")
    right_lines_path = os.path.join(rect_sample_dir, f"right_rectified_lines_{file_suffix}.png")
    cv.imwrite(left_sample_path, left_rectified)
    cv.imwrite(right_sample_path, right_rectified)
    cv.imwrite(left_lines_path, left_rectified_with_lines)
    cv.imwrite(right_lines_path, right_rectified_with_lines)
    
    cv.imshow('Left Rectified Sample', left_rectified)
    cv.imshow('Right Rectified Sample', right_rectified)
    cv.imshow('Left Rectified with Lines', left_rectified_with_lines)
    cv.imshow('Right Rectified with Lines', right_rectified_with_lines)
    cv.waitKey(0)
    cv.destroyAllWindows()
    
    print("Rectified sample images saved to:")
    print(left_sample_path)
    print(right_sample_path)
    print("Rectified images with epipolar lines saved to:")
    print(left_lines_path)
    print(right_lines_path)
    
    print("\n== Stereo Calibration and Rectification Complete ==")
    print("\nAll processing complete. Terminating script.")

if __name__ == "__main__":
    main()