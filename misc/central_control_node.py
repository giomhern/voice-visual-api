# central_control_node.py
import rospy
from std_msgs.msg import String
from eup_stretch.msg import GraspCommand, ArmCommand, NavCommand, VoiceCommand
from threading import Thread
from pynput import keyboard
from time import sleep
import subprocess
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad

from flask import Flask, request, jsonify
from flask_cors import CORS
import json
import os
from datetime import datetime


#####################
###   Utilities   ###
#####################

from std_srvs.srv import Trigger, TriggerRequest

def switch_to_navigation_mode():
    rospy.wait_for_service('switch_to_navigation_mode')

    s = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)
    print(result)
    assert result.success

def switch_to_position_mode():
    rospy.wait_for_service('switch_to_position_mode')

    s = rospy.ServiceProxy('switch_to_position_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)
    print(result)
    assert result.success

def update_vocal_profile():
    file_path = os.path.join(base, 'voice_counter.txt')
    try:
        with open(file_path, 'r') as file:
            counter = int(file.read().strip())
    except FileNotFoundError:
        print("voice_counter file not found, defaulting to 0")
        counter = 0
    
    profile_mapping = {
        0: "surfer_dude",
        1: "surfer_girl",
        2: "business_man",
        3: "business_girl",
        4: "teenager_boy",
        5: "teenager_girl",
        6: "old_man",
        7: "old_woman",
        8: "meditation_woman",
        9: "butler_man",
        10: "robot",
    }
    
    vocal_profile = profile_mapping[counter % 11]
    print("Vocal Profile:", vocal_profile)
    counter += 1
    
    with open(file_path, 'w') as file:
        file.write(str(counter))
    
    return vocal_profile

def read_vocal_profile():
    file_path = os.path.join(base, 'voice_counter.txt')
    try:
        with open(file_path, 'r') as file:
            counter = int(file.read().strip())
    except FileNotFoundError:
        print("voice_counter file not found, defaulting to 0")
        counter = 0
    
    profile_mapping = {
        0: "surfer_dude",
        1: "surfer_girl",
        2: "business_man",
        3: "business_girl",
        4: "teenager_boy",
        5: "teenager_girl",
        6: "old_man",
        7: "old_woman",
        8: "meditation_woman",
        9: "butler_man",
        10: "robot",
    }
    
    vocal_profile = profile_mapping[(counter-1) % 11]
    print("Read Vocal Profile:", vocal_profile)
    
    return vocal_profile

#################
###   Setup   ###
#################

app = Flask(__name__)
CORS(app)

base = os.path.dirname(os.path.realpath(__file__))

# vocal profile selector
vocal_profile = read_vocal_profile()

# loading and serializing customizations
json_file_path = os.path.join(base, 'cc_customizations.json')

parent_of_base = os.path.dirname(base)
archives_folder = os.path.join(parent_of_base, 'archives_CC')

# Create the archives_HL directory if it doesn't exist
if not os.path.exists(archives_folder):
    os.makedirs(archives_folder)

CUSTOMIZATIONS = {}
with open(json_file_path, 'r') as file:
    CUSTOMIZATIONS = json.load(file)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    global CUSTOMIZATIONS, vocal_profile
    data = request.json
    print("Received data:", data)
    CUSTOMIZATIONS = data

    with open(json_file_path, 'w') as file:
        json.dump(CUSTOMIZATIONS, file)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    archived_file_name = f'{timestamp}_control_customizations.json'

    archive_file_path = os.path.join(archives_folder, archived_file_name)
    with open(archive_file_path, 'w') as file:
        json.dump(CUSTOMIZATIONS, file)

    # Set voice profile (iterates through the 11 profiles)
    vocal_profile = update_vocal_profile()
    return jsonify({"message": "Data received successfully"}), 200



class MyKeyListener:
    def __init__(self, abortKey):

        # Recovery Arguments (DEPRECATED)
        # 1: person_nav = 1, aruco_nav = 1,
        # 2: person_nav = 2, aruco_nav = 1,
        # 3: person_nav = 2, aruco_nav = 1,

        self.abortKey = abortKey
        self.grasp_command_pub = rospy.Publisher('grasp_command_topic', GraspCommand, queue_size=10)
        self.arm_command_pub = rospy.Publisher('arm_command_topic', ArmCommand, queue_size=10)
        self.nav_command_pub = rospy.Publisher('nav_command_topic', NavCommand, queue_size=10)
        self.voice_command_pub = rospy.Publisher('voice_command_topic', VoiceCommand, queue_size=10)


    def on_press(self, key):
        try:
            k = key.char
        except:
            k = key.name
        
        print('pressed', k)
        if k == self.abortKey:
            print('ending loop')
            return False
        
        elif k == 't':
            print('stowing arm')
            # self.move.stow_arm()
            arm_msg = ArmCommand()
            arm_msg.command = "stow_arm"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'w':
            print('raise_lift_extend_arm')
            # self.move.position_lift_extend_arm()
            arm_msg = ArmCommand()
            arm_msg.command = "raise_lift_extend_arm"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'e':
            print('position_grab')
            # self.move.grab_bag_stow_arm()
            arm_msg = ArmCommand()
            arm_msg.command = "position_grab"
            self.arm_command_pub.publish(arm_msg)
            sleep(1)
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "grasp_object"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

        elif k == 'y':
            print('manual grab')
            arm_msg = ArmCommand()
            arm_msg.command = "grab"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'r':
            print('lower_lift_extend_arm')
            # self.move.extend_arm_lower_lift()
            arm_msg = ArmCommand()
            arm_msg.command = "lower_lift_extend_arm"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'o':
            # self.move.open_gripper()
            print("open gripper")
            arm_msg = ArmCommand()
            arm_msg.command = "open_gripper"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'c':
            # self.move.open_gripper()
            print("close gripper")
            arm_msg = ArmCommand()
            arm_msg.command = "close_gripper"
            self.arm_command_pub.publish(arm_msg)
        
        elif k == 'h':
            print("head forward")
            arm_msg = ArmCommand()
            arm_msg.command = "head_face_forward"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'x':
            print("base 180")
            arm_msg = ArmCommand()
            arm_msg.command = "base_180"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'left':
            print("base 45 left")
            arm_msg = ArmCommand()
            arm_msg.command = "base_45_left"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'right':
            print("base 45 right")
            arm_msg = ArmCommand()
            arm_msg.command = "base_45_right"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'up':
            print("base forward")
            arm_msg = ArmCommand()
            arm_msg.command = "base_forward"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'down':
            print("base backward")
            arm_msg = ArmCommand()
            arm_msg.command = "base_backward"
            self.arm_command_pub.publish(arm_msg)

        elif k == 'g':
            print('starting grasp...')
            arm_msg = ArmCommand()
            arm_msg.command = "open_gripper"
            self.arm_command_pub.publish(arm_msg)
            sleep(1)
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "grasp_object"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)
            grasp_msg = GraspCommand()
            grasp_msg.command = "grasp_object"
            self.grasp_command_pub.publish(grasp_msg)

        elif k == 'p':
            # preemptively making sure head facing forward to avoid position/nav error
            print("head forward person nav")
            arm_msg = ArmCommand()
            arm_msg.command = "head_face_forward"
            self.arm_command_pub.publish(arm_msg)
            sleep(1)
            print('starting nav towards person')
            nav_msg = NavCommand()
            nav_msg.command = "person"
            nav_msg.movement_speed = 0.3
            nav_msg.stopping_distance = 1.0
            self.nav_command_pub.publish(nav_msg)
            
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "person_navigation"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

        elif k == 'a':
            print('starting nav towards bag drop AR tag')
            nav_msg = NavCommand()
            nav_msg.command = "aruco_bagdrop"
            nav_msg.movement_speed = 0.3
            nav_msg.stopping_distance = 1.0
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "bag_drop_navigation"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.nav_command_pub.publish(nav_msg)
            self.voice_command_pub.publish(voice_msg)

        elif k == 's':
            print('starting nav towards sweet snack AR tag')
            nav_msg = NavCommand()
            nav_msg.command = "aruco_sweet"
            nav_msg.movement_speed = 0.3
            nav_msg.stopping_distance = 1.0
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "snack_navigation"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.nav_command_pub.publish(nav_msg)
            self.voice_command_pub.publish(voice_msg)

        elif k == 'd':
            print('starting nav towards savory snack AR tag')
            nav_msg = NavCommand()
            nav_msg.command = "aruco_savory"
            nav_msg.movement_speed = 0.3
            nav_msg.stopping_distance = 1.0
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "snack_navigation"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.nav_command_pub.publish(nav_msg)
            self.voice_command_pub.publish(voice_msg)

        elif k == "l":
            print("listening to voice command")
            voice_msg = VoiceCommand()
            voice_msg.command = "start"
            voice_msg.name = CUSTOMIZATIONS["firstName"]
            voice_msg.text = "placeholder. this is not LL so no text needed"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

        elif k == "k":
            print("stopping listening")
            voice_msg = VoiceCommand()
            voice_msg.command = "stop"
            voice_msg.name = CUSTOMIZATIONS["firstName"]
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

        elif k == "7":
            print("pre entertainment XAI")
            voice_msg = VoiceCommand()
            voice_msg.command = "XAI"
            voice_msg.XAI_action = "entertainment"
            voice_msg.XAI_level = "minXAI"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

        elif k == "9":
            print("repeat please")
            voice_msg = VoiceCommand()
            voice_msg.command = "LL_speech"
            voice_msg.name = CUSTOMIZATIONS["firstName"]
            voice_msg.text = f"Sorry, {CUSTOMIZATIONS['firstName']}, I didn't quite catch that. Could you please repeat what you said?"
            voice_msg.talking_speed = "moderate"
            voice_msg.vocal_profile = vocal_profile
            self.voice_command_pub.publish(voice_msg)

# '''
# 0: "first_hello": "You introduced yourself as the user's robot assistant.",
# 1: "take_bag": "You took the user's bag.",
# 2: "sit_couch": "You directed the user to go to the couch.",
# 3: "greet_user": "You asked the user how their day went.",
# 4: "respond_greeting": "You talked about the user's day",
# 5: "your_day": "You talked about the day",
# 6: "snack_sequence": "You asked the user for their snack preference",
# 7: "fetching_snack": "You got the user a snack.",
# 8: "handing_snack": "You handed the snack over.",
# 9: "entertainment": "You performed a skit.",
# 10:"ending": "The end."
# '''

    def sleeping_loop(self):
        while True:
            print('awaiting the next command...')
            sleep(5)

    def start(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

if __name__ == '__main__':
    try:
        rospy.init_node('central_control_node')
        abortKey = 'esc'
        listener = MyKeyListener(abortKey)

        # Start the keyboard listener in a separate thread
        listener_thread = Thread(target=listener.start, args=())
        listener_thread.daemon = True  # Set as daemon thread to allow Ctrl+C interruption
        listener_thread.start()

        # Start the sleeping loop in a separate thread
        sleeping_thread = Thread(target=listener.sleeping_loop, args=())
        sleeping_thread.daemon = True
        sleeping_thread.start()

        # Start Flask server in a separate thread
        flask_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False))
        flask_thread.start()

        rospy.spin()  # Keep the central control node running

    except KeyboardInterrupt:
        rospy.loginfo('Interrupt received, shutting down central control node')
