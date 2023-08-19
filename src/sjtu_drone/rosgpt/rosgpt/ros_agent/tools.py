from kor import create_extraction_chain, from_pydantic
from enum import Enum
from typing import Optional, Type
from pydantic import BaseModel, Field
from langchain.tools import BaseTool
from langchain.callbacks.manager import CallbackManagerForToolRun, AsyncCallbackManagerForToolRun
from langchain.chat_models.openai import ChatOpenAI

LLM = ChatOpenAI(temperature=0)

class ActionType(Enum):
    land = "land"
    takeoff = "takeoff"
    move = "move"
    stop = "stop"

class MoveParams(BaseModel):
    linear_speed: float = Field(
        ..., 
        description="The linear speed of the drone in meters per second.", 
        examples=[("Move down for 2 meters at a speed of 0.4 meters per second.", 0.4)],
        ge=0, le=1)
    
    distance: float = Field(
        ..., 
        description="The distance to move in meters.", 
        examples=[("Move down for 2 meters at a speed of 0.4 meters per second.", 2)], 
        ge=-1, le=1)
    
    direction: str = Field(
        ..., 
        description="The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.",
        examples=[("Move down for 2 meters at a speed of 0.4 meters per second.", "down")],
        regex="forward|backward|left|right|up|down")

class Action(BaseModel):
    action: ActionType
    params: Optional[MoveParams]

class ExtractionInput(BaseModel):
    command: str = Field()

schema, validator = from_pydantic(Action) 
extraction_chain = create_extraction_chain(LLM, schema, encoder_or_encoder_class='json', validator=validator)

class CustomCommandToJSON(BaseTool):
    name = "command_to_json"
    description = "useful when you want to convert a single explicit command to a JSON object suitable for a ROS agent. The command must be in English."
    args_schema: Type[BaseModel] = ExtractionInput

    def _run(
        self, command: str, run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return extraction_chain.run(command)
    
    async def _arun(
        self, command: str, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Command to JSON does not support async")