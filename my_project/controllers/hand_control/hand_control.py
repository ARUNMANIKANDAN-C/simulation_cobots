from controller import Supervisor
from flask import Flask, jsonify, request, render_template
import threading

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO with Flask for hand control."""
    
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
        
        # Start the Flask server in a separate thread
        threading.Thread(target=self.start_flask_server, daemon=True).start()
    
    def start_flask_server(self):
        """Start the Flask app to control hand angles via joystick."""
        app = Flask(__name__, template_folder='./templates')

        @app.route('/')
        def index():
            return render_template('joystick.html')  # Serve HTML for the joystick

        @app.route('/update_angles', methods=['POST'])
        def update_angles():
            data = request.get_json()
            for key in self.hand_angles:
                if key in data:
                    self.hand_angles[key] = data[key]
            return jsonify(self.hand_angles)

        @app.route('/get_angles', methods=['GET'])
        def get_angles():
            return jsonify(self.hand_angles)

        app.run(host='0.0.0.0', port=5000)

    def apply_hand_angles(self):
        """Apply the current hand angles to the pedestrian."""
        # Assign the angles fetched from Flask
        self.angles[0][0] = self.hand_angles['leftArmAngle']  
        self.angles[1][0] = self.hand_angles['leftLowerArmAngle']  
        self.angles[2][0] = self.hand_angles['leftHandAngle']  
        self.angles[3][0] = self.hand_angles['rightArmAngle']  
        self.angles[4][0] = self.hand_angles['rightLowerArmAngle']  
        self.angles[5][0] = self.hand_angles['rightHandAngle']  

    def run(self):
        """Main loop."""
        while self.step(32) != -1:  # Run Webots simulation at 32 ms steps
            self.apply_hand_angles()
