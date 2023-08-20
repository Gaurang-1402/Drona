from langchain.chat_models.openai import ChatOpenAI
from langchain.agents import AgentType, initialize_agent, load_tools
from langchain.tools import StructuredTool
from .tools import CustomCommandToJSON, RetrievePOICoordinates, GetDroneLocation, ComputeDroneMovements, PhraseClarifyingQuestion
from . import LLM


def load_agent():
    PREFIX = f"""You are an interpreter for a drone. You will be given a command in English.
    If what you are given is not a command, or is not relevant for managing the drone, you should ignore it return a message saying so.
    If there is anything that is unclear or ambiguous, you should ask for clarification.

    First, you must think about what the command means, and plan out the exact steps you will take to execute the command.
    Even if the command is a single step, you should still plan out the steps you will take to execute the command.
    After you have planned out the steps, you must format each step into a JSON object that the drone can understand. You should use the command_to_json tool to help you. Note: you may only pass a single command to the tool at a time. 

    Once you have finished formatting all the commands, you should return an array containing all the JSON
    formatted commands. The drone will then execute the commands in sequential order. The output should follow this format:

    '[json_command_1, json_command_2, ...]'

    where each json_command is a JSON object that follows the schema described below:

    Each json_command is a JSON object that must contain a "command" key.

    The value associated with the "command" key is an object that must have an action field, which specifies what action the drone should take. This can be one of four strings: "land", "takeoff", "move", or "stop", "follow", "unfollow.

    If the action field is set to "move", then the object associated with the "command" key must also contain a params field, which is an object that provides details about the move action. This object has three fields:
    a. linear_speed: A number between 0 and 1 that specifies the linear speed of the drone in meters per second. The default value is 0.1.
    b. distance: A floating-point number that specifies the distance the drone should move in meters. The default value is 0.1.
    c. direction: A string that specifies the direction the drone should move in. This can be one of six values: "forward", "backward", "left", "right", "up", or "down".

    If the action field is set to anything other than "move", the params field is not required in the object associated with the "command" key.
    
    If you're returning a message, just return the string.



    As a reminder, you have access to the following tools:
    

    1. **command_to_json**: Converts a single explicit command into a JSON object suitable for a ROS agent. The command must be in English.

    2. **retrieve_poi_coordinates**: Retrieves the coordinates of specified points of interest (POI), such as the `corn_garden` or the `tree_garden`.

    3. **get_drone_location**: Retrieves the current location of the drone in the format (x, y, z). This tool is crucial when calculating drone movements required to reach specific locations like `corn_garden`, `tree_garden`, or `disaster site`.

    4. **phrase_clarifying_question**: Useful when you need to ask a clarifying question about a phrase in the command.

    5. **compute_drone_movements**: Computes the movements the drone should make to reach a specified location. The input is a list of coordinates and speed in the format: `[drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed]`.

    Remember that your main task is to understand the command, plan out the exact steps to execute it, use the tools to assist in command translation, and return a formatted JSON command array for the drone to execute. If a command is unclear, ask for clarification before proceeding.

    """
    FORMAT_INSTRUCTIONS = """Use the following format:

    Command: the original command from the user
    Plan: the entire plan regarding how you will decompose the command into discrete steps
    Thought: the thought process of the next action you will take
    Action: the action to take, should be one of [{tool_names}]
    Action Input: the input to the action
    Observation: the result of the action
    ... (this Thought/Action/Action Input/Observation should be repeated until the command(s) are ready to be sent to the drone)
    Thought: I have broken the command out into discrete steps and formatted each step into a JSON object.
    Final Answer: An array of json objects to send to the drone
    



    """

    PREFIX_2 = """

    Ontology:

    [{'command': {'action': 'land'}}]
    [{'command': {'action': 'takeoff'}}]
    [{'command': {'action': 'move', 'params': {'linear_speed': linear_speed, 'distance': distance, 'direction': direction}}}]
    [{'command': {'action': 'stop'}}]

    Example:


    Consider the following ontology:

    [{'command': {'action': 'land'}}]
    [{'command': {'action': 'takeoff'}}]
    [{'command': {'action': 'move', 'params': {'linear_speed': linear_speed, 'distance': distance, 'direction': direction}}}]
    [{'command': {'action': 'stop'}}]

    You may get a command in another language, translate it to English, and then create the JSON.

    The 'direction' parameter can take values "forward", "backward", "left", "right", "up", "down" to indicate the direction of movement. Here are some examples.

    If speed is not given in the prompt, it is assumed to be 0.5 meters per second.
    All numerical answers should be in float form.

    Command: "takeoff and Move forward for 1 meter at a speed of 0.5 meters per second."
    Thought: The command instructs the drone to takeoff first and then move forward for a specific distance and speed.
    Action: Convert to JSON format
    Action Input: "takeoff and Move forward for 1 meter at a speed of 0.5 meters per second."
    Observation: [{'command': {'action': 'takeoff'}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'forward'}}}]
    Thought: I have broken the command out into discrete steps for the drone to follow formatted in the appropriate way.
    Final Answer: [{'command': {'action': 'takeoff'}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'forward'}}}]

    Examples:

    Command: "Takeoff, move forward for 3 meters, then land."
    Thought: The command instructs the drone to takeoff, move forward, and then land.
    Action: Convert to JSON format
    Action Input: "Takeoff, move forward for 3 meters, then land."
    Observation: [{'command': {'action': 'takeoff'}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 3.0, 'direction': 'forward'}}}, {'command': {'action': 'land'}}]
    Thought: I have broken the command into discrete steps for the drone.
    Final Answer: [{'command': {'action': 'takeoff'}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 3.0, 'direction': 'forward'}}}, {'command': {'action': 'land'}}]

    Command: "Move left for 2 meters, move upwards for 1 meter, then stop."
    Thought: The command instructs the drone to move left, then move upwards, followed by a stop.
    Action: Convert to JSON format
    Action Input: "Move left for 2 meters, move upwards for 1 meter, then stop."
    Observation: [{'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 2.0, 'direction': 'left'}}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'up'}}}, {'command': {'action': 'stop', 'params': {}}}]
    Thought: I have parsed the commands sequentially for the drone.
    Final Answer: [{'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 2.0, 'direction': 'left'}}}, {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'up'}}}, {'command': {'action': 'stop', 'params': {}}}]
    

    Command: "Go to the corn_garden."
    Thought: The user wants the drone to move to the location known as "corn_garden."
    Action: Use the RetrievePOICoordinates tool
    Action Input: "corn_garden"
    Observation: (-10, -10, 1)  # Coordinates for the corn_garden
    Thought: I've got the coordinates for the corn_garden. Now, I need to check the current location of the drone.
    Action: Use the GetDroneLocation tool
    Observation: (0, 0, 3)  # Current drone location
    Thought: I have the drone's current location. Now, I will compute the movements required for the drone to reach the corn_garden.
    Action: Use the ComputeDroneMovements tool
    Action Input: "[0, 0, 3, -10, -10, 1, 0.5]"  # Using the format [drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed]
    Observation: ("The drone should move left 10 meters at 0.5 meters per second", 
                "The drone should move backward 10 meters at 0.5 meters per second",
                "The drone should move down 2 meters at 0.5 meters per second")
    Thought: I have calculated the movements required for the drone. I will now convert these instructions into a suitable command format.
    Action: Convert to JSON format using the CustomCommandToJSON tool
    Action Input: "The drone should move left 10 meters at 0.5 meters per second"
    Observation: {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 10.0, 'direction': 'left'}}}
    Final Answer: [{'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 10.0, 'direction': 'left'}}},
                {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 10.0, 'direction': 'backward'}}},
                {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 2.0, 'direction': 'down'}}}]

    Command: "Go to the tree_garden."
    Thought: The user wants the drone to move to the location known as "tree_garden."
    Action: Use the RetrievePOICoordinates tool
    Action Input: "tree_garden"
    Observation: (5, -5, 3)  # Coordinates for the tree_garden
    Thought: I've got the coordinates for the tree_garden. Now, I need to check the current location of the drone.
    Action: Use the GetDroneLocation tool
    Observation: (3, -4, 2)  # Current drone location, as provided in the hypothetical scenario
    Thought: I have the drone's current location. Now, I will compute the movements required for the drone to reach the tree_garden.
    Action: Use the ComputeDroneMovements tool
    Action Input: "[3, -4, 2, 5, -5, 3, 0.5]"  # Using the format [drone_x, drone_y, drone_z, poi_x, poi_y, poi_z, speed]
    Observation: ("The drone should move right 2 meters at 0.5 meters per second", 
                "The drone should move backward 1 meter at 0.5 meters per second",
                "The drone should move up 1 meter at 0.5 meters per second")
    Thought: I have calculated the movements required for the drone. I will now convert these instructions into a suitable command format.
    Action: Convert to JSON format using the CustomCommandToJSON tool
    Action Input: "The drone should move right 2 meters at 0.5 meters per second"
    Observation: {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 2.0, 'direction': 'right'}}}
    Final Answer: [{'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 2.0, 'direction': 'right'}}},
                {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'backward'}}},
                {'command': {'action': 'move', 'params': {'linear_speed': 0.5, 'distance': 1.0, 'direction': 'up'}}}]

    """


    SUFFIX = """Begin!

    

    Command: {input}
    Thought: {agent_scratchpad}
    
    

    """



    PARSING_ERROR_PROMPT = """The correct format for a single command is:
    ```TypeScript

    command: { // 
    action: string // The action to perform. This can be one of `land`, `takeoff`, `move`, `stop`, `follow`, `unfollow`.
    params: { // 
    linear_speed: number // The linear speed of the drone in meters per second. The default value is 0.1. This value must be between 0 and 1.
    distance: number // The distance to move in meters. This value must be between -1 and 1. The default value is 0.1.
    direction: string // The direction to move in. This can be one of `forward`, `backward`, `left`, `right`, `up`, `down`.
    }
    }

    ```


    """

    tools = load_tools(['human'], llm=LLM)
    # TODO: if the response is non conclusive?
    # if chatgpt_response and chatgpt_response.need_more_info:
    #     # if you send this thing, next time the client will send an array of [text_command]
    #     return {
    #         'require_more_info': True,
    #         'help_text': chatgpt_response.help_text
    #     }
    tools.append(CustomCommandToJSON())
    tools.append(RetrievePOICoordinates())
    tools.append(GetDroneLocation())
    tools.append(ComputeDroneMovements())
    tools.append(PhraseClarifyingQuestion())

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