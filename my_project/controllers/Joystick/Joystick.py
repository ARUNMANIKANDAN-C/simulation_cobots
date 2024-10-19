from controller import Supervisor, Keyboard
from flask import Flask, jsonify, request, render_template
import threading
import math

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO with Flask for hand control and keyboard inputs."""
    
    def __init__(self):
        super(Pedestrian, self).__init__()
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.05
        self.speed = 1
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle"
        ]
        self.hand_angles = {
            'leftArmAngle': 0,
            'leftLowerArmAngle': 0,
            'leftHandAngle': 0,
            'rightArmAngle': 0,
            'rightLowerArmAngle': 0,
            'rightHandAngle': 0
        }
        
        self.point_list = ["5 0", "5 0.3"]
        self.angles = [...]  # Omitted for brevity, same as before
        self.height_offsets = [...]  # Omitted for brevity, same as before
        
        self.key_board = Keyboard()
        _ = self.key_board.enable(16)
        self.my_time = 0
        self.angle = 0

        # Start the Flask server in a separate thread
        threading.Thread(target=self.start_flask_server, daemon=True).start()

    def apply_hand_angles(self):
        """Apply the current hand angles to the pedestrian."""
        self.angles[0][0] = self.hand_angles['leftArmAngle']
        self.angles[1][0] = self.hand_angles['leftLowerArmAngle']
        self.angles[2][0] = self.hand_angles['leftHandAngle']
        self.angles[3][0] = self.hand_angles['rightArmAngle']
        self.angles[4][0] = self.hand_angles['rightLowerArmAngle']
        self.angles[5][0] = self.hand_angles['rightHandAngle']

    def Convert(self):
        temp = self.point_list[-1]
        X, Y = temp.split(' ')
        X = float(X)
        Y = float(Y)
        return X, Y

    def update_position(self, dx, dy):
        """Helper function to update pedestrian's position."""
        X, Y = self.Convert()
        self.point_list[-2] = self.point_list[-1]
        self.point_list[-1] = f"{X + dx} {Y + dy}"
        self.Start_up()

    def keyboardvalue(self):
        """Handle keyboard inputs for moving the pedestrian."""
        key = self.key_board.getKey()
        if key == 315:  # Move up
            self.update_position(0, 0.2)
        elif key == 316:  # Move left
            self.update_position(-0.2, 0)
        elif key == 314:  # Move right
            self.update_position(0.2, 0)
        elif key == 317:  # Move down
            self.update_position(0, -0.2)

    def Start_up(self):
        """Initialize and compute waypoints and distances."""
        self.time_step = int(self.getBasicTimeStep())
        self.number_of_waypoints = len(self.point_list)
        self.waypoints = []
        for i in range(0, self.number_of_waypoints):
            self.waypoints.append([float(v) for v in self.point_list[i].split()])

        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")

        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        self.waypoints_distance = []
        for i in range(self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            z = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            dist = math.sqrt(x * x + z * z)
            self.waypoints_distance.append(dist if i == 0 else self.waypoints_distance[i - 1] + dist)
        
        self.my_time = self.getTime()

    def run(self):
        """Main loop to update pedestrian's movement and animations."""
        self.Start_up()
        while not self.step(self.time_step) == -1:
            time = self.my_time
            self.keyboardvalue()
            current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))

            for i in range(self.BODY_PARTS_NUMBER):
                current_angle = self.angles[i][current_sequence] * (1 - ratio) + self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)

            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
            distance = time * self.speed
            relative_distance = distance % self.waypoints_distance[-1]

            for i in range(self.number_of_waypoints):
                if self.waypoints_distance[i] > relative_distance:
                    break

            distance_ratio = relative_distance / self.waypoints_distance[0] if i == 0 else (relative_distance - self.waypoints_distance[i - 1]) / (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
            x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + (1 - distance_ratio) * self.waypoints[i][0]
            z = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + (1 - distance_ratio) * self.waypoints[i][1]
            root_translation = [z, x, self.ROOT_HEIGHT + self.current_height_offset]

            angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0], self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1])
            rotation = [0, 1, 0, angle]

            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)

    def start_flask_server(self):
        """Start Flask server in a separate thread."""
        app.run(host='0.0.0.0', port=5000, debug=False)


# Flask App Setup
app = Flask(__name__, template_folder='./templates')

@app.route('/')
def index():
    return render_template('joystick.html')  # Serve the joystick HTML page

@app.route('/update_angles', methods=['POST'])
def update_angles():
    data = request.get_json()
    for key in controller.hand_angles:
        if key in data:
            controller.hand_angles[key] = data[key]
    controller.apply_hand_angles()
    return jsonify(controller.hand_angles)

@app.route('/get_angles', methods=['GET'])
def get_angles():
    return jsonify(controller.hand_angles)

# Start the pedestrian controller
controller = Pedestrian()
controller.run()
