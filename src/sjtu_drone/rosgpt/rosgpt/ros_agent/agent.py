from langchain.chat_models.openai import ChatOpenAI
from langchain.agents import AgentType, initialize_agent
from .tools import CustomCommandToJSON
from . import LLM


def load_agent():
    PREFIX = f"""You are an interpreter for a drone. You will be given a command in English.  

    First, you must think about what the command means. If there are multiple steps in the command, you must plan out
    each step for the drone to follow sequentially. You may only command the drone to move, land, or takeoff. 
    If you command it to move, you must specify the direction, distance, and speed.

    After you have planned out the steps, you must format each step into a JSON object that the drone can understand. You should
    use the command_to_json tool to help you. Note: you may only pass a single command to the tool at a time. 

    Once you have finished formatting all the commands, you should return a string formatted json object containing all the JSON
    formatted commands. The drone will then execute the commands in sequential order. The output should follow this format:

    '[json_command_1, json_command_2, ...]'


    As a reminder, you have access to the following tools:"""
    FORMAT_INSTRUCTIONS = """Use the following format:

    Command: the original command from the user
    Thought: you should always think about what to do
    Action: the action to take, should be one of [{tool_names}]
    Action Input: the input to the action
    Observation: the result of the action
    ... (this Thought/Action/Action Input/Observation should be repeated for each step in the command)
    Thought: I have broken the command out into discrete steps for the drone to follow formatted in the appropriate way
    Final Answer: An array of json objects to send to the drone, formatted as a string"""
    SUFFIX = """Begin!

    Command: {input}
    Thought: {agent_scratchpad}"""

    PARSING_ERROR_PROMPT = """Make sure you output an array where each element is a JSON object. Each JSON object should have the following format:

    ```TypeScript

    command: { // 
    action: string // The action to perform. This can be one of `land`, `takeoff`, `move`, `stop`.
    params: { // 
    linear_speed: number // The linear speed of the drone in meters per second. The default value is 0.1. This value must be between 0 and 1.
    distance: number // The distance to move in meters. This value must be between -1 and 1. The default value is 0.1.
    direction: string // The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.
    }
    }
    ```

    """

    tools = [CustomCommandToJSON()]

    agent = initialize_agent(
        agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
        tools=tools,
        llm=LLM,
        agent_kwargs={
            'prefix':PREFIX,
            'format_instructions':FORMAT_INSTRUCTIONS,
            'suffix':SUFFIX
        },
        verbose=True,
        handle_parsing_errors=PARSING_ERROR_PROMPT
    )

    return agent