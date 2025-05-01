import cv2
import numpy as np
import glob
import os

# Calibration board parameters
CHECKERBOARD = (8, 11)  # Inner corners (rows, cols) for 9x12 board
SQUARE_SIZE = 30.0  # Square size in mm
ANNOTATIONS_SUBDIR = "chessboard_annotations"  # Subdirectory for annotated images
PARAMETERS_SUBDIR = "parameters"  # Subdirectory for stereo parameters

# Prompt user for input and output directories and .yaml files
left_input_dir = input("Enter the input directory for left camera images (e.g., left_images): ").strip()
right_input_dir = input("Enter the input directory for right camera images (e.g., right_images): ").strip()
output_dir = input("Enter the output directory for results (e.g., calibration_results): ").strip()
left_yaml = input("Enter the path to left camera .yaml file (e.g., calibration_results/parameters/left.yaml): ").strip()
right_yaml = input("Enter the path to right camera .yaml file (e.g., calibration_results/parameters/right.yaml): ").strip()

# Validate input directories
if not os.path.isdir(left_input_dir):
    raise ValueError(f"Left input directory '{left_input_dir}' does not exist.")
if not os.path.isdir(right_input_dir):
    raise ValueError(f"Right input directory '{right_input_dir}' does not exist.")

# Create output directory and subdirectories
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

for subdir in [ANNOTATIONS_SUBDIR, PARAMETERS_SUBDIR]:
    subdir_path = os.path.join(output_dir, subdir)
    if not os.path.exists(subdir_path):
        os.makedirs(subdir_path)

# Termination criteria for sub-pixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D coordinates of chessboard corners)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Arrays to store object points and image points for both cameras
objpoints = []  # 3D points (shared for both cameras)
imgpoints_left = []  # 2D points for left camera
imgpoints_right = []  # 2D points for right camera

# Load mono calibration parameters from .yaml files
def load_camera_params(yaml_file):
    fs = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Could not open {yaml_file}")
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    image_width = int(fs.getNode("image_width").real())
    image_height = int(fs.getNode("image_height").real())
    fs.release()
    return camera_matrix, dist_coeffs, (image_width, image_height)

# Load left and right camera parameters
try:
    mtx_left, dist_left, size_left = load_camera_params(left_yaml)
    mtx_right, dist_right, size_right = load_camera_params(right_yaml)
except FileNotFoundError as e:
    print(e)
    exit(1)

# Validate image sizes
if size_left != size_right:
    print("Warning: Left and right camera image sizes differ.")
image_size = size_left  # Use left camera size for calibration

# Load synchronized image pairs
left_images = sorted(glob.glob(os.path.join(left_input_dir, "*.png")))  # Assumes .png
right_images = sorted(glob.glob(os.path.join(right_input_dir, "*.png")))  # Assumes .png

if len(left_images) != len(right_images):
    raise ValueError("Number of left and right images must match.")

# Ensure we have image pairs
image_pairs = list(zip(left_images, right_images))
if not image_pairs:
    raise ValueError("No image pairs found. Check image paths.")

# Process each image pair
for idx, (left_fname, right_fname) in enumerate(image_pairs):
    print(f"Processing pair {idx + 1}/{len(image_pairs)}: {left_fname}, {right_fname}")
    
    # Read grayscale images
    img_left = cv2.imread(left_fname, cv2.IMREAD_GRAYSCALE)
    img_right = cv2.imread(right_fname, cv2.IMREAD_GRAYSCALE)
    
    if img_left is None or img_right is None:
        print(f"Failed to load pair: {left_fname}, {right_fname}")
        continue

    # Step 1: Apply CLAHE to enhance contrast (address glossiness)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    left_contrast = clahe.apply(img_left)
    right_contrast = clahe.apply(img_right)

    # Step 2: Apply Gaussian Blur to reduce noise
    left_blurred = cv2.GaussianBlur(left_contrast, (5, 5), 0)
    right_blurred = cv2.GaussianBlur(right_contrast, (5, 5), 0)

    # Step 3: Apply Median Blur for additional noise reduction
    left_denoised = cv2.medianBlur(left_blurred, 3)
    right_denoised = cv2.medianBlur(right_blurred, 3)

    # Step 4: Apply adaptive thresholding for robust corner detection
    left_thresh = cv2.adaptiveThreshold(
        left_denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    right_thresh = cv2.adaptiveThreshold(
        right_denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )

    # Step 5: Morphological opening to remove small noise artifacts
    kernel = np.ones((3, 3), np.uint8)
    left_cleaned = cv2.morphologyEx(left_thresh, cv2.MORPH_OPEN, kernel)
    right_cleaned = cv2.morphologyEx(right_thresh, cv2.MORPH_OPEN, kernel)

    # Step 6: Find chessboard corners in both images
    ret_left, corners_left = cv2.findChessboardCorners(
        left_cleaned,
        CHECKERBOARD,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    ret_right, corners_right = cv2.findChessboardCorners(
        right_cleaned,
        CHECKERBOARD,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret_left and ret_right:
        # Step 7: Refine corners to sub-pixel accuracy
        corners_left = cv2.cornerSubPix(left_denoised, corners_left, (11, 11), (-1, -1), criteria)
        corners_right = cv2.cornerSubPix(right_denoised, corners_right, (11, 11), (-1, -1), criteria)
        
        # Store points
        objpoints.append(objp)
        imgpoints_left.append(corners_left)
        imgpoints_right.append(corners_right)

        # Draw corners for visualization
        img_left_color = cv2.cvtColor(img_left, cv2.COLOR_GRAY2BGR)
        img_right_color = cv2.cvtColor(img_right, cv2.COLOR_GRAY2BGR)
        img_left_with_corners = cv2.drawChessboardCorners(img_left_color, CHECKERBOARD, corners_left, ret_left)
        img_right_with_corners = cv2.drawChessboardCorners(img_right_color, CHECKERBOARD, corners_right, ret_right)
        
        # Save annotated images
        left_output_path = os.path.join(output_dir, ANNOTATIONS_SUBDIR, f"left_annotated_{os.path.basename(left_fname)}")
        right_output_path = os.path.join(output_dir, ANNOTATIONS_SUBDIR, f"right_annotated_{os.path.basename(right_fname)}")
        cv2.imwrite(left_output_path, img_left_with_corners)
        cv2.imwrite(right_output_path, img_right_with_corners)
        print(f"Saved: {left_output_path}, {right_output_path}")
    else:
        print(f"Chessboard not detected in pair: {left_fname}, {right_fname}")

# Perform stereo calibration if valid data is collected
if len(objpoints) > 0:
    # Step 8: Stereo calibration using imported intrinsics
    flags = cv2.CALIB_FIX_INTRINSIC  # Fix imported intrinsics
    ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_left,
        imgpoints_right,
        mtx_left,
        dist_left,
        mtx_right,
        dist_right,
        image_size,
        criteria=criteria,
        flags=flags
    )
    print(f"Stereo reprojection error: {ret_stereo:.3f} pixels")
    
    # Step 9: Stereo rectification
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        mtx_left, dist_left, mtx_right, dist_right, image_size, R, T
    )
    print("Stereo calibration and rectification completed.")

    # Save reprojection annotated images
    print("Generating and saving reprojection images...")
    for idx, (left_fname, right_fname) in enumerate(image_pairs):
        if idx >= len(objpoints):
            continue  # Skip pairs where chessboard was not detected
        
        # Read original images in color for visualization
        img_left = cv2.imread(left_fname)
        img_right = cv2.imread(right_fname)
        
        if img_left is None or img_right is None:
            print(f"Failed to load pair for reprojection: {left_fname}, {right_fname}")
            continue

        # Compute per-image pose using solvePnP
        # Left camera
        ret, rvec_left, tvec_left = cv2.solvePnP(objpoints[idx], imgpoints_left[idx], mtx_left, dist_left)
        if not ret:
            print(f"Failed to solvePnP for left image: {left_fname}")
            continue
        proj_left = cv2.projectPoints(objpoints[idx], rvec_left, tvec_left, mtx_left, dist_left)[0].reshape(-1, 2)

        # Right camera
        ret, rvec_right, tvec_right = cv2.solvePnP(objpoints[idx], imgpoints_right[idx], mtx_right, dist_right)
        if not ret:
            print(f"Failed to solvePnP for right image: {right_fname}")
            continue
        proj_right = cv2.projectPoints(objpoints[idx], rvec_right, tvec_right, mtx_right, dist_right)[0].reshape(-1, 2)

        # Draw reprojected corners (red) and detected corners (green)
        img_left_color = img_left.copy()
        img_right_color = img_right.copy()
        
        # Draw detected corners (green)
        for corner in imgpoints_left[idx]:
            center = (int(corner[0][0]), int(corner[0][1]))
            cv2.circle(img_left_color, center, 5, (0, 255, 0), -1)
        for corner in imgpoints_right[idx]:
            center = (int(corner[0][0]), int(corner[0][1]))
            cv2.circle(img_right_color, center, 5, (0, 255, 0), -1)
        
        # Draw reprojected corners (red)
        for point in proj_left:
            center = (int(point[0]), int(point[1]))
            cv2.circle(img_left_color, center, 5, (0, 0, 255), -1)
        for point in proj_right:
            center = (int(point[0]), int(point[1]))
            cv2.circle(img_right_color, center, 5, (0, 0, 255), -1)
        
        # Save reprojected images
        left_reproj_path = os.path.join(
            output_dir, ANNOTATIONS_SUBDIR, f"left_reprojected_{os.path.basename(left_fname)}"
        )
        right_reproj_path = os.path.join(
            output_dir, ANNOTATIONS_SUBDIR, f"right_reprojected_{os.path.basename(right_fname)}"
        )
        cv2.imwrite(left_reproj_path, img_left_color)
        cv2.imwrite(right_reproj_path, img_right_color)
        print(f"Saved reprojected images: {left_reproj_path}, {right_reproj_path}")

    # Save stereo calibration results
    yaml_path = os.path.join(output_dir, PARAMETERS_SUBDIR, "stereo.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix_left", mtx_left)
    fs.write("distortion_coefficients_left", dist_left)
    fs.write("camera_matrix_right", mtx_right)
    fs.write("distortion_coefficients_right", dist_right)
    fs.write("rotation_matrix", R)
    fs.write("translation_vector", T)
    fs.write("essential_matrix", E)
    fs.write("fundamental_matrix", F)
    fs.write("rectification_transform_left", R1)
    fs.write("rectification_transform_right", R2)
    fs.write("projection_matrix_left", P1)
    fs.write("projection_matrix_right", P2)
    fs.write("disparity_to_depth_matrix", Q)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.release()
    print(f"Saved stereo calibration parameters to {yaml_path}")
else:
    print("No valid image pairs for calibration.")

# Release resources
cv2.destroyAllWindows()
