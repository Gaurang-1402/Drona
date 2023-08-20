from kor import create_extraction_chain, from_pydantic
from typing import Optional, Type, Tuple
from pydantic import BaseModel, Field
from langchain.tools import BaseTool
from langchain.callbacks.manager import CallbackManagerForToolRun, AsyncCallbackManagerForToolRun
from langchain.chat_models.openai import ChatOpenAI
from . import LLM


class MoveParams(BaseModel):
    linear_speed: float = Field(
        ...,
        description="The linear speed of the drone in meters per second. The default value is 0.3. This value must be between 0 and 1.", 
        ge=0, le=1)
    
    distance: float = Field(
        ...,
        description="The distance to move in meters. This value must be a float. The default value is 0.1.")
    
    direction: str = Field(
        ..., 
        description="The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.",
        enum=["forward", "backward", "left", "right", "up", "down"])

class Command(BaseModel):
    action: str = Field(
        ...,
        description="The action to perform. This can be one of `land`, `takeoff`, `move`, `stop`.",
        enum=["land", "takeoff", "move", "stop"]
    )
    params: Optional[MoveParams]


class ExtractionInput(BaseModel):
    command: str = Field(...)

schema, validator = from_pydantic(
    Command,
    examples=[
        ("Land the drone.", {"action": "land"}),
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
    description = ("useful when you want to retrieve the coordinates of the garden or the kitchen.")
    
    args_schema: Type[BaseModel] = ExtractionInput
    def _run(
        self,
        location: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        poi_coordinates = {
            'garden': (10, 30, 2),
            'kitchen': (60, 20, 1)
        }
        try:
            coords = poi_coordinates[location]
            return coords
        except KeyError:
            return(f"Invalid location: {location}. Please try another tool.")
                     
    async def _arun(
        self,
        location: str,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Retrieve POI Coordinates does not support async")

class GetDroneLocation(BaseTool):
    name = "get_drone_location"
    description = ("useful when you want to retrieve the current location of the drone.")

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        drone_location = (10, 20, 1)

        return drone_location
                     
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
    description = ("useful when you need to ask a clarifying question about a phrase in the command, or simply tell the user that you don't understand the command.")
    args_schema: Type[BaseModel] = QuestionInput
    return_direct = True

    def _run(
        self,
        question: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return question


# TODO: Implement this tool  
# class GetDroneOrientation(BaseTool):
#     name = "get_drone_orientation"
#     description = ("useful when you want to retrieve the current orientation of the drone.")
#     args_schema: Type[BaseModel] = ExtractionInput

#     def _run(
#         self,
#         run_manager: Optional[CallbackManagerForToolRun] = None
#     ) -> str:
#         """Use the tool."""
#         drone_orientation = 0

#         return drone_orientation
                     
#     async def _arun(
#         self,
#         run_manager: Optional[AsyncCallbackManagerForToolRun] = None
#     ) -> str:
#         """Use the tool asynchronously."""
#         raise NotImplementedError("Get Drone Orientation does not support async")


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
        # Coordinates could be in the form '[drone_x, drone_y, poi_x, poi_y]'
        drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed = eval(coordinates)

        # Calculate x axis movement (left/right)
        if drone_x < poi_x:
            x_axis_movement = f"The drone should move right {poi_x - drone_x} meters at {speed} meters per second"
        elif drone_x > poi_x:
            x_axis_movement = f"The drone should move left {drone_x - poi_x} meters at {speed} meters per second"

        # Calculate y axis movement (forward/backward)
        if drone_y < poi_y:
            y_axis_movement = f"The drone should move forward {poi_y - drone_y} meters at {speed} meters per second"
        elif drone_y > poi_y:
            y_axis_movement = f"The drone should move backward {drone_y - poi_y} meters at {speed} meters per second"

        # Calculate z axis movement (up/down)
        if drone_z < poi_z:
            z_axis_movement = f"The drone should move up {poi_z - drone_z} meters at {speed} meters per second"
        elif drone_z > poi_z:
            z_axis_movement = f"The drone should move down {drone_z - poi_z} meters at {speed} meters per second"

        return x_axis_movement, y_axis_movement, z_axis_movement