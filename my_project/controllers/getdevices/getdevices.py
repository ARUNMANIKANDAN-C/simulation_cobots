from controller import Robot

# Create a Robot instance
robot = Robot()

# Get the number of devices
num_devices = robot.getNumberOfDevices()

# Print the names of all devices
print("Devices in the robot:")
for i in range(num_devices):
    device = robot.getDeviceByIndex(i)
    print(f"Name: {device.getName()}, Type: {type(device).__name__}")
