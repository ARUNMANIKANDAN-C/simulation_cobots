import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import json
import math
app = Flask(__name__)
socketio = SocketIO(app, async_mode='eventlet')

# Initialize angles dictionary
dic = { 
    'leftArmAngle': 0,
    'leftLowerArmAngle': 0,
    'leftHandAngle': 0,
    'rightArmAngle': 0,
    'rightLowerArmAngle': 0,
    'rightHandAngle': 0
}

# Route to serve the HTML page
@app.route('/')
def index():
    return render_template('index.html')

# Handle joystick data from the client
@socketio.on('joystick_movement')
def handle_joystick_movement(data):
    # Log and update angle data based on client input
    #print("Received joystick data:", json.dumps(data, indent=2))
    if 'id' in data and 'angle' in data:
        dic[data['id']] = math.radians(data['angle'])
    # Send a confirmation back to the client
    socketio.emit('status', {'status': 'Data received'})

# Route to get the current angles
@app.route('/get_angles', methods=["GET"])
def get_angles():
    return jsonify(dic)

if __name__ == '__main__':
    # Run the Flask server with Socket.IO in eventlet mode
    socketio.run(app, host='0.0.0.0', port=5000)
