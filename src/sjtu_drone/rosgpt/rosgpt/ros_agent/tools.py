from kor import create_extraction_chain, from_pydantic
from typing import Optional, Type
from pydantic import BaseModel, Field
from langchain.tools import BaseTool
from langchain.callbacks.manager import CallbackManagerForToolRun, AsyncCallbackManagerForToolRun
from langchain.chat_models.openai import ChatOpenAI


LLM = ChatOpenAI(temperature=0)


class MoveParams(BaseModel):
    linear_speed: float = Field(
        ...,
        description="The linear speed of the drone in meters per second.",
        ge=0, le=1)

    distance: float = Field(
        ...,
        description="The distance to move in meters.",
        ge=-1, le=1)

    direction: str = Field(
        ...,
        description="The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.",
        enum=["forward", "backward", "left", "right", "up", "down"])


class Command(BaseModel):
    action: str = Field(
        ...,
        description="The action to perform. This can be one of `land`, `takeoff`, `move`, `stop`.",
        enum=["land", "takeoff", "move", "stop"])
    params: Optional[MoveParams]


class ExtractionInput(BaseModel):
    command: str = Field(...)


class CustomCommandToJSON(BaseTool):
    name = "command_to_json"
    description = ("useful when you want to convert a single explicit "
                   "command to a JSON object suitable for a ROS agent. "
                   "The command must be in English.")
    args_schema: Type[BaseModel] = ExtractionInput

    def __init__(self):
        schema, validator = from_pydantic(
            Command,
            examples=[
                ("Land the drone.", {"action": "land"}),
                ("Takeoff the drone.", {"action": "takeoff"}),
                ("Move down for 2 meters at a speed of 0.4 meters per second.",
                 {"action": "move",
                  "params": {"linear_speed": 0.4, "distance": 2, "direction": "down"}})])
        self.extraction_chain = create_extraction_chain(LLM, schema, encoder_or_encoder_class='json', validator=validator)

    def _run(
        self,
        command: str,
        run_manager: Optional[CallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool."""
        return self.extraction_chain.run(command)

    async def _arun(
        self,
        command: str,
        run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("Command to JSON does not support async")