import cv2
import cv2.aruco as aruco
import numpy as np
import csv

# Camera parameters (assumed to be known or calibrated)
camera_matrix = np.array([[982.36, 0, 634.88],
                          [0, 981.23, 356.47],
                          [0, 0, 1]])
dist_coeffs = np.array([0.1, -0.25, 0, 0, 0])


# Function to calculate distance, yaw, pitch, and roll
def calculate_3d_info(rvec, tvec):
    # Calculate distance to the camera
    distance = np.linalg.norm(tvec)

    # Calculate yaw, pitch, and roll angles
    R, _ = cv2.Rodrigues(rvec)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    roll = np.arctan2(R[2, 1], R[2, 2])

    return distance, yaw, pitch, roll


# Load the Aruco dictionary and detector parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
parameters = aruco.DetectorParameters()

# Open the video
cap = cv2.VideoCapture('C:/Users/User/Desktop/Tello/Video.mp4')

# Check if the video is opened correctly
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# CSV file to write the output
with open('aruco_output.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Frame ID", "QR ID", "QR 2D", "Distance", "Yaw", "Pitch", "Roll"])

    # Process each frame
    frame_id = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are detected
        if ids is not None:
            for i in range(len(ids)):
                # Get the corner points
                corner_points = corners[i][0].tolist()

                # Draw the detected markers with green rectangular frame
                frame = aruco.drawDetectedMarkers(frame, corners)

                # Estimate pose of each marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)

                # Calculate 3D information
                distance, yaw, pitch, roll = calculate_3d_info(rvec, tvec)

                # Draw the pose of the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Write to CSV
                writer.writerow([frame_id, ids[i][0], corner_points, distance, np.degrees(yaw), np.degrees(pitch),
                                 np.degrees(roll)])

                # Display the marker ID on the frame
                cv2.putText(frame, f"ID: {ids[i][0]}", tuple(map(int, corner_points[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2, cv2.LINE_AA)

        # Display the frame
        cv2.imshow('Frame', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_id += 1

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
