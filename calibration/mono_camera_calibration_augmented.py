import cv2
import numpy as np
import glob
import os
import random

# Calibration board parameters
CHECKERBOARD = (8, 11)  # Inner corners (rows, cols) for 9x12 board
SQUARE_SIZE = 30.0  # Square size in mm
ANNOTATIONS_SUBDIR = "chessboard_annotations"  # Subdirectory for annotated images
RECTIFICATION_SUBDIR = "rectification_sample"  # Subdirectory for rectified sample
PARAMETERS_SUBDIR = "parameters"  # Subdirectory for camera parameters

# Prompt user for input and output directories
input_dir = input("Enter the input directory containing images (e.g., left_images): ").strip()
output_dir = input("Enter the output directory for results (e.g., calibration_results): ").strip()
camera_name = input("Enter camera name for .yaml file (e.g., left or right): ").strip()

# Validate input directory
if not os.path.isdir(input_dir):
    raise ValueError(f"Input directory '{input_dir}' does not exist.")

# Create output directory and subdirectories
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

for subdir in [ANNOTATIONS_SUBDIR, RECTIFICATION_SUBDIR, PARAMETERS_SUBDIR]:
    subdir_path = os.path.join(output_dir, subdir)
    if not os.path.exists(subdir_path):
        os.makedirs(subdir_path)

# Termination criteria for sub-pixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D coordinates of chessboard corners)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Arrays to store object points and image points
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load grayscale images from input directory
images = glob.glob(os.path.join(input_dir, "*.png"))  # Assumes .png; adjust if needed
if not images:
    raise ValueError(f"No images found in '{input_dir}'. Check path and file extensions.")

# Process each image
for idx, fname in enumerate(images):
    print(f"Processing image {idx + 1}/{len(images)}: {fname}")
    img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Failed to load {fname}")
        continue

    # Step 1: Apply CLAHE to enhance contrast (address glossiness)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    contrast_enhanced = clahe.apply(img)

    # Step 2: Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(contrast_enhanced, (5, 5), 0)

    # Step 3: Apply Median Blur for additional noise reduction
    denoised = cv2.medianBlur(blurred, 3)

    # Step 4: Apply adaptive thresholding for robust corner detection
    thresh = cv2.adaptiveThreshold(
        denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )

    # Step 5: Morphological opening to remove small noise artifacts
    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # Step 6: Find chessboard corners
    ret, corners = cv2.findChessboardCorners(
        cleaned,
        CHECKERBOARD,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        # Step 7: Refine corners to sub-pixel accuracy
        corners = cv2.cornerSubPix(denoised, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw corners on original image for visualization
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_with_corners = cv2.drawChessboardCorners(img_color, CHECKERBOARD, corners, ret)
        
        # Save annotated image
        output_path = os.path.join(output_dir, ANNOTATIONS_SUBDIR, f"annotated_{os.path.basename(fname)}")
        cv2.imwrite(output_path, img_with_corners)
        print(f"Saved annotated image: {output_path}")
    else:
        print(f"Chessboard not detected in {fname}")

# Perform calibration if valid data is collected
if len(objpoints) > 0:
    # Step 8: Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img.shape[::-1], None, None
    )
    print(f"Reprojection error: {ret:.3f} pixels")

    # Step 9: Save camera parameters to .yaml
    yaml_path = os.path.join(output_dir, PARAMETERS_SUBDIR, f"{camera_name}.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("image_width", img.shape[1])
    fs.write("image_height", img.shape[0])
    fs.release()
    print(f"Saved camera parameters to {yaml_path}")

    # Step 10: Select and rectify a random image
    valid_images = [img for img, corners in zip(images, imgpoints) if corners is not None]
    if valid_images:
        random_image = random.choice(valid_images)
        print(f"Rectifying sample image: {random_image}")
        img = cv2.imread(random_image, cv2.IMREAD_GRAYSCALE)
        
        # Undistort the image
        h, w = img.shape
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(img, mtx, dist, None, new_camera_matrix)
        
        # Convert to BGR for saving
        undistorted_color = cv2.cvtColor(undistorted, cv2.COLOR_GRAY2BGR)
        
        # Save rectified sample
        rectified_path = os.path.join(output_dir, RECTIFICATION_SUBDIR, f"rectified_{os.path.basename(random_image)}")
        cv2.imwrite(rectified_path, undistorted_color)
        print(f"Saved rectified sample: {rectified_path}")
else:
    print("No valid images for calibration.")

# Release resources
cv2.destroyAllWindows()
