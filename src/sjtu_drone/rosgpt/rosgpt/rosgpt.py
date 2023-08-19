import json
import os
import threading
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Union

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
from .ros_agent.tools import CustomCommandToJSON

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)

LLM = OpenAI(temperature=0)


@tool("drone_location")
def get_drone_location():
    """Gets the current drone location.

    Returns:
        tuple: The current drone location in the form (x, y, z).
    """
    return ROSGPTNode.get_loc()

@tool("poi_location")
def get_poi_location(point_of_interest: str) -> tuple:
    """Gets the current point of interest location.

    Returns:
        tuple: The current point of interest location in the form (x, y, z).
    """
    poi_map = {
        'garden': (0, 0, 0),
        'greenhouse': (1, 1, 1),
        'kitchen': (2, 2, 2)
        }
    return poi_map[point_of_interest]

def load_agent():
    tools = [CustomCommandToJSON()]

    agent = initialize_agent(
        tools, LLM, agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION, verbose=True
    )
    return agent



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
            'position_data', 
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
        self.position = (msg.point.x, msg.point.y, msg.point.z)

    def get_loc(self):
        """
        Return the latest position data received by the subscriber.

        Returns
        -------
        tuple or None
            The latest position data in the form (x, y, z), or None if no position data has been received.
        """
        return self.position
    
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

    # def askGPT(self, command) -> str:
    #     """
    #     Send a text command to the LangChain agent and return the response.
    #     Args:
    #         command (str): The text command to be sent to the GPT-3 model.
    #     Returns:
    #         str: The response from the GPT-3 model as a JSON string.
    #     """


    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the GPT-3 model and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the GPT-3 model response as a JSON string.
        """

        text_command = request.form['text_command']
        chatgpt_response = self.agent.run(text_command)
        print(chatgpt_response)

        if chatgpt_response is None:
            return {'error': 'An error occurred while processing the request'}

        return json.loads(chatgpt_response)

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
