import json
import os
import threading
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Union

import json
import openai
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
import rclpy
from ament_index_python import get_package_share_directory
from flask import Flask, request, send_from_directory
from flask_cors import CORS
from flask_restful import Api, Resource
from geometry_msgs.msg import Pose
from langchain.agents import AgentType, initialize_agent, load_tools
from langchain.llms import OpenAI
from langchain.output_parsers import PydanticOutputParser
from langchain.prompts import PromptTemplate
from langchain.tools import tool
from pydantic import BaseModel, Field, validator
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
import subprocess

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from ros_agent.agent import load_agent

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app, origins=['*'])

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)


# Now you can use the openai_api_key variable to authenticate with the OpenAI API


# Initialize a threading lock for synchronizing access to shared resources
# when multiple threads are involved
spin_lock = threading.Lock()


# Create a separate threading lock for synchronizing access to the TTS engine
tts_lock = threading.Lock()


class ROSGPTNode(Node):
    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        super().__init__('chatgpt_ros2_node')
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)
        
        # Subscribe to the 'position_data' topic
        self.subscription = self.create_subscription(
            Pose, 
            '/drone/gt_pose', 
            self.position_callback, 
            10)
        
        self.subscription
        self.position = None  # Initialize the position attribute to None

    def publish_message(self, message):
        """
        Publish the given message to the 'voice_cmd' topic.

        Parameters
        ----------
        message : str
            The message to be published.
        """
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

    def position_callback(self, msg):
        """
        Callback function that gets executed whenever a new position message is received.

        Parameters
        ----------
        msg : PointStamped
            The received message containing position data.
        """
        self.position = (msg.position.x, msg.position.y, msg.position.z)

    def get_loc(self):
        """
        Return the latest position data received by the subscriber.

        Returns
        -------
        tuple or None
            The latest position data in the form (x, y, z), or None if no position data has been received.
        """
        return self.position
    
def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    """
    Process the chatbot's response and publish it to the 'voice_cmd' topic.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    chatgpt_ros2_node.publish_message(chatgpt_response) # Publish the chatbot's response using the ROS2 node
    # If use_executors flag is True, use SingleThreadedExecutor
    if use_executors:
        executor = SingleThreadedExecutor()# Create a new executor for each request 
        executor.add_node(chatgpt_ros2_node) # Add the node to the executor
        executor.spin_once()#  Spin the executor once
        executor.remove_node(chatgpt_ros2_node) # Remove the node from the executor
    # If use_executors flag is False, use spin_lock to synchronize access
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)

    
class ROSGPTProxy(Resource):
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node):
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node
        self.agent = load_agent()

    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the GPT-3 model and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the GPT-3 model response as a JSON string.
        """

        text_command = request.form['text_command']

        print('subhmx', text_command)

        if text_command.startswith('[') and text_command.endswith(']'):
            text_command = json.loads(text_command)
        else:
            text_command = [text_command]
        
        print(text_command)
        chatgpt_response = self.agent.run(text_command)

        print('subhmx122', chatgpt_response)

        if not chatgpt_response:
            return {'error': 'An error occurred while processing the request'}
        elif chatgpt_response == "Agent stopped due to iteration limit or time limit." or chatgpt_response == "[]":
            return {'error': 'An error occurred while processing the request'}
        elif chatgpt_response[0] != '[':
            return {
                'require_more_info': True,
                'help_text': chatgpt_response
            }
        else:
            print('subhmx123: response was successful')
            threading.Thread(
                target=process_and_publish_chatgpt_response, 
                args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
            return {
                'response': "Captain, your command is being executed now!",
                'chatgpt_response': chatgpt_response
            }

@app.route('/')
def index():
    webapp_dir = Path(get_package_share_directory('rosgpt')) / 'webapp'
    return send_from_directory(str(webapp_dir), 'index.html')

def main():
    rclpy.init(args=None)
    chatgpt_ros2_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros2_node,))
    app.run(debug=False, host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
