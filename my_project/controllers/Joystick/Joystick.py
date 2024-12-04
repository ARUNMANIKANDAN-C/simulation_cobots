# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container."""
from controller import Supervisor, Keyboard, Robot, GPS
supervisor = Supervisor()
# Create the Robot instance
robot = Robot()

import requests
import math

headers = {'Accept': 'application/json'}

class Pedestrian (Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.05
        self.speed = 1
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.hand_angles = {
            'leftArmAngle': 0,
            'leftLowerArmAngle': 0,
            'leftHandAngle': 0,
            'rightArmAngle': 0,
            'rightLowerArmAngle': 0,
            'rightHandAngle': 0
        }
        self.height_offsets = [  # those coefficients are empirical coeffici  ents which result in a realistic walking gait
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [  
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        
        
        Supervisor.__init__(self)
        # Enable keyboard
        
        self.key_board=Keyboard()
        self.point_list = ["5 0" , "5 0.3" ]
        _=Keyboard.enable(self.key_board,16)
        
        self.my_time=0
        self.angle=0
        

    def Start_up(self):
        self.time_step = int(self.getBasicTimeStep())
        self.number_of_waypoints = len(self.point_list)
        self.waypoints = []
        for i in range(0, self.number_of_waypoints):
            self.waypoints.append([])
            self.waypoints[i].append(float(self.point_list[i].split()[0]))
            self.waypoints[i].append(float(self.point_list[i].split()[1]))
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        
        
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # compute waypoints distance
        self.waypoints_distance = []
        for i in range(0, self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            z = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x * x + z * z))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x * x + z * z))
        self.my_time = self.getTime()

    def run(self):
        #self.gps_value()
        self.Start_up()
        while not self.step(self.time_step) == -1:
            time=self.my_time
            if self.keyboardvalue():
                current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
                # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
                # / 'total distance between way-point(X) and way-point(X+1)'
                ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                    int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))

                for i in range(0, self.BODY_PARTS_NUMBER):
                    current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                        self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                    self.joints_position_field[i].setSFFloat(current_angle)

                # adjust height
                self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                    self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

                # move everything
                distance = time * self.speed
                relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                    self.waypoints_distance[self.number_of_waypoints - 1]

                for i in range(0, self.number_of_waypoints):
                    if self.waypoints_distance[i] > relative_distance:
                        break

                distance_ratio = 0
                if i == 0:
                    distance_ratio = relative_distance / self.waypoints_distance[0]
                else:
                    distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                        (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
                x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                    (1 - distance_ratio) * self.waypoints[i][0]
                z = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                    (1 - distance_ratio) * self.waypoints[i][1]
                    
                    
                root_translation = [z, x, self.ROOT_HEIGHT + self.current_height_offset]
                angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0],
                                self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1])
                
                rotation = [0, 0, 1, self.angle]

                self.root_translation_field.setSFVec3f(root_translation)
                self.root_rotation_field.setSFRotation(rotation)

    def Convert(self):
        temp=self.point_list[-1]
        X,Y=temp.split(' ')
        X=float(X)
        Y=float(Y)
        return X,Y
    def hand_control(self):
        self.temp_joint=[self.getSelf().getField(self.joint_names[i]) for i in range(0,6)]
        key = list(self.hand_angles.keys())
        """Apply the current hand angles to the pedestrian."""
        try:
            r = requests.get('http://127.0.0.1:5000/get_angles', headers=headers)
            r.raise_for_status()  # Raise an error for bad responses

            # Update hand angles
            self.hand_angles = r.json()
            for i in range(0, 6):
                # Convert keys to a list to access by index
                self.temp_joint[i].setSFFloat(self.hand_angles[key[i]])

        except requests.RequestException as e:
            pass
            #print(f"Failed to retrieve data: {e}")
            
    """def gps_value(self):

        # Define timestep for simulation updates
        timestep = int(robot.getBasicTimeStep())

        # Initialize GPS devices for left and right hands
        left_hand_gps = robot.getDevice("leftHandSlot")
        right_hand_gps = robot.getDevice("rightHandSlot")

        # Enable the GPS devices
        left_hand_gps.enable(timestep)
        right_hand_gps.enable(timestep)

        # Initialize the keyboard for user input
        keyboard = Keyboard()
        keyboard.enable(timestep)

        print("Press 'L' to display the left hand position or 'R' for the right hand position.")

        while supervisor.step(timestep) != -1:
            # Get the latest key press
            key = keyboard.getKey()
            
            # Check for specific key presses
            if key == ord('L'):
                # Get and print the left hand position
                left_hand_position = left_hand_gps.getValues()
                print("Left Hand Position:", left_hand_position)
                
            elif key == ord('R'):
                # Get and print the right hand position
                right_hand_position = right_hand_gps.getValues()
                print("Right Hand Position:", right_hand_position)
                
            # Optional: Add other keys to perform different actions"""
                
    # Keyboard values.
    def keyboardvalue(self):
            key = self.key_board.getKey()
            
            if key==315:
                X,Y=self.Convert()
                self.point_list[-2]=self.point_list[-1]
                change=str(X)+' '+ str(Y+0.2)
                self.point_list[-1]=change
                self.Start_up()
                self.angle=0
                return 1

            elif key==316:
                X,Y=self.Convert()
                self.point_list[-2]=self.point_list[-1]
                change=str(X-0.2)+' '+ str(Y)
                self.point_list[-1]=change
                self.Start_up()
                self.angle=-1.57
                return 1
            elif key==314:
                X,Y=self.Convert()
                self.point_list[-2]=self.point_list[-1]
                change=str(X+0.2)+' '+ str(Y)
                self.point_list[-1]=change
                self.Start_up()
                self.angle=1.57
                return 1
            elif key==317:
                X,Y=self.Convert()
                self.point_list[-2]=self.point_list[-1]
                change=str(X)+' '+ str(Y-0.2)
                self.point_list[-1]=change
                self.Start_up()
                self.angle=3.142
                return 1
            elif key==76 :

                data = {"hand":"left"}
                response = requests.post('http://127.0.0.1:5000/get_L_pick', data=data)
                return 0
            
            elif key==82:
                data = {"hand":"right"}
                response = requests.post('http://127.0.0.1:5000/get_R_pick', data=data)
                return 0
            else:
                self.hand_control()
                return 0

controller = Pedestrian()

controller.run()