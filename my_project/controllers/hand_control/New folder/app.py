from flask import Flask, request, jsonify, render_template
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# Initialize dictionary to store joystick angles for each control
joystick_data = {
    'leftArmAngle': 0,
    'leftLowerArmAngle': 0,
    'leftHandAngle': 0,
    'rightArmAngle': 0,
    'rightLowerArmAngle': 0,
    'rightHandAngle': 0
}

@app.route('/', methods=['POST', 'GET'])
def update_angles():
    """Update joystick angles based on data from the frontend."""
    if request.method == 'POST':
        data = request.get_json()  # Get JSON data from request
        if data:
            # Update angles in joystick_data dictionary
            joystick_data.update(data)
            print(data)  # Log the received data
            return jsonify({'status': 'success', 'message': 'Angles updated successfully!'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'Invalid data!'}), 400
    return render_template('index.html')

@app.route('/get_angles', methods=['GET'])
def get_angles():
    """Retrieve the latest joystick angles."""
    return jsonify(joystick_data)

if __name__ == '__main__':
    app.run(debug=True)
