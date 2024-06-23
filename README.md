# Aruco Marker Detection and Pose Estimation

This Python script utilizes OpenCV and Aruco markers for detecting markers in a video stream, estimating their pose in 3D space, and saving the results to a CSV file.

## Overview

The script performs the following steps:
1. Loads camera calibration parameters.
2. Detects Aruco markers in each frame of a video.
3. Estimates the 3D pose (translation and rotation) of each detected marker.
4. Calculates the distance, yaw, pitch, and roll of each marker.
5. Saves the results to a CSV file.

## Requirements

- Python 3.x
- OpenCV (with Aruco module)
- NumPy

## Installation

To install the required libraries, you can use pip:
* `pip install opencv-python-headless numpy`

## Usage
1. Ensure you have a calibrated camera. Update the camera_matrix and dist_coeffs in the script with your camera's calibration data.
2. Place your video file in the specified path (update cap = cv2.VideoCapture('C:/Users/User/Desktop/Tello/Video.mp4') if necessary).
3. Run the script: `python aruco_marker_detection.py`

## Output
* The script will display the video with detected Aruco markers and their poses.
* A CSV file (aruco_output.csv) will be generated containing the following columns:
* Frame ID
* QR ID
* QR 2D (corner points)
* Distance
* Yaw (in degrees)
* Pitch (in degrees)
* Roll (in degrees)

## Notes
1. Ensure your camera is properly calibrated for accurate pose estimation.
2. Adjust the marker size (0.05 in aruco.estimatePoseSingleMarkers) according to your actual marker size.
3. The script stops if the 'q' key is pressed while the video is being displayed.
