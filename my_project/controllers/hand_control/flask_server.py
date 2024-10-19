from flask import Flask, jsonify, request, render_template

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('joystick.html')  # Serve HTML for the joystick

@app.route('/update_angles', methods=['POST'])
def update_angles():
    data = request.get_json()
    for key in app.config['hand_angles']:
        if key in data:
            app.config['hand_angles'][key] = data[key]
    return jsonify(app.config['hand_angles'])

@app.route('/get_angles', methods=['GET'])
def get_angles():
    return jsonify(app.config['hand_angles'])

def start_flask_server(hand_angles=[]):
    #app.config['hand_angles'] = hand_angles
    app.run(host='0.0.0.0', port=5000)

start_flask_server()