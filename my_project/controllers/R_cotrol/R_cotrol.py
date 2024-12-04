from controller import Robot
from flask import request
import requests

# Create the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Enable GPS
gps = robot.getDevice('gps')
gps.enable(timestep)
"""
# Enable the IMU for orientation (if needed)
imu = robot.getDevice('imu')
imu.enable(timestep)
"""
# Main loop
while robot.step(timestep) != -1:
    # Get position from GPS
    position = gps.getValues()  # Returns [x, y, z] in world coordinates
    x, y, z = position
    
    # Print position data
    print(f"Position - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")

    # Additional IMU data processing (optional)
    # ll, pitch, yaw = imu.getRollPitchYaw()
    # Print or use IMU data as needed

    # Robot control logic here
