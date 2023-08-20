from kor import create_extraction_chain, from_pydantic
from typing import Any, Optional, Type, Tuple
from pydantic import BaseModel, Field
from langchain.tools import BaseTool
from langchain.callbacks.manager import CallbackManagerForToolRun, AsyncCallbackManagerForToolRun
from langchain.chat_models.openai import ChatOpenAI
from . import LLM
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import traceback
import copy
import requests


class MoveParams(BaseModel):
    linear_speed: float = Field(
        ...,
        description="The linear speed of the drone in meters per second. The default value is 0.3. This value must be between 0 and 1.", 
        ge=0, le=1)
    
    distance: float = Field(
        ...,
        description="The distance to move in meters. This value must be a float. The default value is 0.4")
    
    direction: str = Field(
        ..., 
        description="The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.",
        enum=["forward", "backward", "left", "right", "up", "down"])

class Command(BaseModel):
    action: str = Field(
        ...,
        description="The action to perform. This can be one of `land`, `takeoff`, `move`, `stop`, `follow`, `unfollow`.",
        enum=["land", "takeoff", "move", "stop", "follow", "unfollow"]
    )
    params: Optional[MoveParams]


class ExtractionInput(BaseModel):
    command: str = Field(...)

schema, validator = from_pydantic(
    Command,
    examples=[
        ("Land the drone.", {"action": "land"}),
        ("Follow the human.", {"action": "follow"}),
        ("Unfollow the human.", {"action": "unfollow"}),
        ("Takeoff the drone.", {"action": "takeoff"}),
        ("Move down for 2 meters at a speed of 0.4 meters per second.",
            {"action": "move",
            "params": {"linear_speed": 0.4, "distance": 2, "direction": "down"}}),
        ("Move forward for 2 meters at a speed of 0.4 meters per second.",
            {"action": "move",
            "params": {"linear_speed": 0.4, "distance": 2, "direction": "forward"}}),
        ("Stop the drone.", {"action": "stop"})])
extraction_chain = create_extraction_chain(LLM, schema, encoder_or_encoder_class='json', validator=validator)

class CustomCommandToJSON(BaseTool):
    name = "command_to_json"
    description = ("useful when you want to convert a single explicit "
                   "command to a JSON object suitable for a ROS agent. "
                   "The command must be in English.")
    args_schema: Type[BaseModel] = ExtractionInput

    def _run(
        self,
        command: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return extraction_chain.run(command)

    async def _arun(
        self,
        command: str,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Command to JSON does not support async")

class POIInput(BaseModel):
    location: str = Field(..., description="The location of the point of interest (POI).")

class RetrievePOICoordinates(BaseTool):
    name = "retrieve_poi_coordinates"
    description = ("useful when you want to retrieve the coordinates of the corn_garden or the tree_garden.")
    
    args_schema: Type[BaseModel] = ExtractionInput
    def _run(
        self,
        location: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        if location[0] == "'":
            location = location.lstrip("'")

        if location[-1] == "'":
            location = location.rstrip('"')

        if location[-1] == "\n":
            location = location.rstrip("\n")

        poi_coordinates = {
            'corn_garden': (-10, -10, 1),
            'tree_garden': (5, -5, 3)
        }
        try:
            coords = poi_coordinates[location]
            return coords
        except KeyError:
                # Capture the traceback as a string
            tb_str = traceback.format_exc()

            # Print or do something with the traceback string
            return(f"Error Received: {tb_str}\n Invalid location: {location}. Please try another tool.")
                     
    async def _arun(
        self,
        location: str,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Retrieve POI Coordinates does not support async")


class DroneLocationInput(BaseModel):
    location: str = Field(..., description="The location of the drone.")


class GetDroneLocation(BaseTool):

    name = "get_drone_location"
    description = ("useful when you want to retrieve the current location of the drone."
                   "The location is returned as a tuple of the form (x, y, z)."
                   "The tool must always be used when calculating drone movements when required to go to a specific location like corn garden or tree garden or disaster site."
                   "get_drone_location must be used before compute_drone_movements.")

    def _run(
        self,
        location: DroneLocationInput,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""

        try:
            # Make a GET request to the specified URL
            response = requests.get('http://localhost:5000/get_drone_location')

            # Check if the response status code is 200 (OK)
            if response.status_code == 200:
                drone_location = response.json()  # Parse the JSON response
                print("Drone location: ", drone_location)
                return drone_location
            else:
                # Handle non-200 status codes (e.g., 404, 500, etc.)
                print(f"Error {response.status_code}: {response.text}")
                return None

        except requests.RequestException as e:
            # Handle potential exceptions (e.g., network issues)
            print2(f"Request error: {e}")
            return None
                     
    async def _arun(
        self,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Get Drone Location does not support async")

class QuestionInput(BaseModel):
    question: str = Field(..., description="The question to ask.")


class PhraseClarifyingQuestion(BaseTool):
    name = "phrase_clarifying_question"
    description = ("useful when you need to ask a clarifying question about a phrase in the command.")
    args_schema: Type[BaseModel] = QuestionInput
    return_direct = True

    def _run(
        self,
        question: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return question


class DroneToPOIInput(BaseModel):
    coordinates: str = Field(
        ..., 
        description= ("The coordinates of the drone, the poi coordinates, and the speed in the form "
                      "[drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed].")
    )

class ComputeDroneMovements(BaseTool):
    name = "compute_drone_movements"
    description = ("useful when you want to compute the movements the drone should take to reach a certain "
                   "location, given a list of coordinates and speed in the form " 
                   "[drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed].")
    args_schema: Type[BaseModel] = DroneToPOIInput

    def _run(
        self,
        coordinates: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> Tuple[str, str]:
        """Use the tool."""

        # Convert the coordinates string to a tuple
        # Coordinates could be in the form '[drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed]'
        
        drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed = eval(coordinates)

        x_axis_movement, y_axis_movement, z_axis_movement = "", "", ""

        # Calculate x axis movement (forward/backward)
        if drone_x < poi_x:
            x_axis_movement = f"The drone should move forward {poi_x - drone_x} meters at {speed} meters per second"
        elif drone_x > poi_x:
            x_axis_movement = f"The drone should move backward {drone_x - poi_x} meters at {speed} meters per second"

        # Calculate y axis movement (left/right)
        if drone_y < poi_y:
            y_axis_movement = f"The drone should move left {poi_y - drone_y} meters at {speed} meters per second"
        elif drone_y > poi_y:
            y_axis_movement = f"The drone should move right {drone_y - poi_y} meters at {speed} meters per second"

        # Calculate z axis movement (up/down)
        if drone_z < poi_z:
            z_axis_movement = f"The drone should move up {poi_z - drone_z} meters at {speed} meters per second"
        elif drone_z > poi_z:
            z_axis_movement = f"The drone should move down {drone_z - poi_z} meters at {speed} meters per second"

        return x_axis_movement, y_axis_movement, z_axis_movement