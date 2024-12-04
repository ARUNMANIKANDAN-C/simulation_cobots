from controller import Supervisor
import random
import math
import csv

# Initialize the supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

# Define movement range and settings
z_position = 1.05
x_min, x_max = 0.3, 0.65
y_min, y_max = -0.5, 0.5
object_size = 0.1
min_distance = object_size * 1.5

# Setup for logging without timestamp
csv_file = "object_positions.csv"
header = ["Object", "X", "Y", "Z"]
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(header)

# Function to calculate the distance between points
def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def is_position_valid(new_position, existing_positions):
    for pos in existing_positions:
        if calculate_distance(new_position, pos) < min_distance:
            return False
    return True

# Access children nodes of the Robot
children_field = supervisor.getSelf().getField("children")
num_children = children_field.getCount()
positions = {}

# Loop through children and find SolidBox nodes
for i in range(num_children):
    child_node = children_field.getMFNode(i)
    if child_node.getTypeName() == "SolidBox":
        box_name = child_node.getField("name").getSFString()
        positions[box_name] = child_node.getField("translation").getSFVec3f()
# Main loop to randomly move objects within specified range
while supervisor.step(time_step) != -1:
    for i in range(num_children):
        child_node = children_field.getMFNode(i)
        
        if child_node.getTypeName() != "SolidBox":
            continue

        box_name = child_node.getField("name").getSFString()

        # Find valid new position
        valid_position = False
        while not valid_position:
            new_x = random.uniform(x_min, x_max)
            new_y = random.uniform(y_min, y_max)
            new_position = [new_x, new_y, z_position]

            if is_position_valid(new_position, positions.values()):
                valid_position = True
                positions[box_name] = new_position
                child_node.getField("translation").setSFVec3f(new_position)

                # Log position to CSV without timestamp
                with open(csv_file, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow([box_name, new_x, new_y, z_position])
