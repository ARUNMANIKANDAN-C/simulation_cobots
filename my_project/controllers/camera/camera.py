from controller import Robot, Camera, Display

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()
display = robot.getDevice('display')
while robot.step(32) != -1:
    data = camera.getImage()
    if data:
        ir = display.imageNew(data, Display.BGRA, width, height)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)

"""import time
from controller import Robot, Camera

# Initialize the robot and camera
robot = Robot()
camera = robot.getDevice("camera_name")  # Replace "camera_name" with your actual camera name

# Enable the camera
camera_sampling_period = 32  # Sampling period in milliseconds
camera.enable(camera_sampling_period)

# Main loop
time_step = int(robot.getBasicTimeStep())
while robot.step(time_step) != -1:
    # Capture the current image
    image = camera.getImage()
    if image:
        # Process the image here (e.g., using OpenCV if desired)
        print("Image captured from the camera")

    # Other code to handle robot operations can go here

import cv2
from controller import Robot, Camera
import numpy as np
# Initialize the Robot instance
robot = Robot()

# Define the time step (default is 32 ms for Webots)
time_step = int(robot.getBasicTimeStep())

# Enable the camera
camera = robot.getDevice("camera")
camera.enable(time_step)

# Set the interval to 60 seconds
display_interval = 60
last_display_time = time.time()

# Run the controller
while robot.step(time_step) != -1:
    current_time = time.time()
    
    # Check if 60 seconds have passed
    if current_time - last_display_time >= display_interval:
        # Capture the camera image
        image = camera.getImage()

        if image:
            # Convert Webots image data to OpenCV format
            width, height = camera.getWidth(), camera.getHeight()
            img_array = bytearray(image)
            img_cv = cv2.imdecode(np.frombuffer(img_array, np.uint8), cv2.IMREAD_COLOR)
            img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGRA2BGR)  # Convert to RGB

            # Display the image using OpenCV
            cv2.imshow("Webots Camera", img_cv)
            cv2.waitKey(1)  # Wait briefly to update the display

            # Update the last display time
            last_display_time = current_time
"""