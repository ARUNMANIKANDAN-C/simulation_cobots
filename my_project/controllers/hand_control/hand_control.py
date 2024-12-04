from controller import Supervisor
import requests
import time

headers = {'Accept': 'application/json'}

class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO with direct hand control."""

    def __init__(self):
        super(Pedestrian, self).__init__()
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

        # Set the time step (in seconds)
        self.time_step = 1.0 / 60.0  # 60 times per second

    def apply_hand_angles(self):
        """Apply the current hand angles to the pedestrian."""
        try:
            r = requests.get('http://127.0.0.1:5000/get_angles', headers=headers)
            r.raise_for_status()  # Raise an error for bad responses

            # Update hand angles
            self.hand_angles = r.json()
            #print(f"Response: {self.hand_angles}")

            # Update angles based on received data
            for joint_name in self.joint_names:
                joint_node = self.getFromDef(joint_name)  # Get the joint node
                if joint_node is not None:
                    rotation_vector = self.get_joint_rotation(joint_name)
                    joint_node.getField('rotation').setSFRotation(rotation_vector)

        except requests.RequestException as e:
            pass
            #print(f"Failed to retrieve data: {e}")

    def get_joint_rotation(self, joint_name):
        """Return the rotation vector for a given joint based on its name."""
        # Convert the angle in degrees to radians
        angle = self.hand_angles[joint_name] * (3.14159 / 180.0)
        # Rotation around the Y-axis as an example
        return (0, 1, 0, angle)

    def run(self):
        """Run the main control loop."""
        while self.step(int(self.time_step * 1000)) != -1:  # Webots expects time in milliseconds
            self.apply_hand_angles()
            time.sleep(self.time_step)  # Sleep to maintain 60 Hz rate

# To run the simulation, create an instance of Pedestrian and call run()
pedestrian = Pedestrian()
pedestrian.run()
