import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Daedalus(Node):
    def __init__(self):
        super().__init__('chatbot')

        self.intentions = {
            r'(hello|hi|hey|hi there|hiya)': 'greeting',
            r'(bye|goodbye|see you)': 'farewell',
            r'\b(a?r?i?a?d?n?e)\b': 'ariadne',
            r'\b(m[ie]n[o0]t[ao]u?r)\b': 'minotaur',
            r'sun|son|sin|su': 'sun',
            r'wings| wengs| wuings': 'wings',
        }

        self.destinations = {
            'ariadne': {'x': 3.80, 'y': 0.50, 'z': 0.0},
            'minotaur': {'x': 1.5, 'y': 1.5, 'z': 0.0},
            'sun': {'x': 1.5, 'y': -1.5, 'z': 0.0},
            'wings': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        }

        self.intention = None
        self.destination = None
        self.publisher_ = self.create_publisher(Pose, '/enqueue', 10)

    def go_to(self):
        pose_msg = Pose()
        pose_msg.position.x = self.destination['x']
        pose_msg.position.y = self.destination['y']
        pose_msg.position.z = self.destination['z']
        self.publisher_.publish(pose_msg)
        print(f'OK! Heading to {self.intention} - Coordinates (x:{pose_msg.position.x}, y:{pose_msg.position.y}, z:{pose_msg.position.z})')

    def end_chat(self):
        print('Goodbye!')
        exit()

    def identify_intention(self, text):
        for expression, intention in self.intentions.items():
            match = re.search(expression, text, re.IGNORECASE)
            if match:
                return intention
        return None

    def perform_action(self, intention):
        if intention:
            if intention == 'greeting':
                print('Hi Icarus! I am Daedalus your father. Where you want you go?')

            elif intention == 'farewell':
                return self.end_chat()

            else:
                self.destination = self.destinations[self.intention]
                self.go_to()

        else:
            print('Sorry, I did not understand what you said.')

    def start_conversation(self):
        while True:
            text = input("You: ")
            self.intention = self.identify_intention(text)
            self.perform_action(self.intention)

def main(args=None):
    rclpy.init(args=args)
    chatbot = Daedalus()
    chatbot.start_conversation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
