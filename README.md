
# Drona 🤖✈️

Video link: https://www.youtube.com/embed/hX7KutK7DyA

[![image.png](https://i.postimg.cc/Gtd4k5D9/image.png)](https://www.youtube.com/watch?v=hX7KutK7DyA)


## Motivation

You may have come accross plenty of projects where autonomous agents are integrated into computers. Often, these agents are equipped with limited tools, restricting them to interfacing with your emails, Excel sheets, or a codebase.

But what if we gave these agents with tools to interact with the real world in the form of robots?

Inspired by this idea, we embarked on a mission to embed an autonomous agent into one of today's most prevalent commercial robots: drones. And we decided to call it Drona, short for drone agent, and also inspired by the most intelligent teacher in Indian mythology: Dronacharya.

On a basic level, Drona is a drone control software that replaces keyboards and gaming joysticks with natural language commands. However, more ambitiously, it introduces a pivotal new component to be integrated in every robot: an "emotional brain", enabling it to interpret and even empathize with the user based on conversation. 

Drona can also enhance the safety of the drones. Today drones embedded with software and hardware that breach privacy. Manufacturers often employ these to track and even disable drones if they're repurposed for illegal activities, such as weapons or espionage. This scenario gives rise to a compromise, where individual privacy is sacrificed for societal safety. Drona challenges this paradigm, offering a system where both safety and ethical considerations are prioritized by the agent. 

With the ease of communication, Drona breaks down linguistic barriers, thus broadening its market potential. Whether you're a farmer in rural India, pr a rescue worker in Japan your drone now understands you. With Drona's, language is no barrier. By simplifying the user interface, we are targeting a global audience.

![image](https://github.com/Gaurang-1402/Drona/assets/71042887/4250104b-85cc-499d-84d5-f58247ed182c)


## Applications 🌾

We'd like to highlight Drona’s utility in two domains: agriculture and disaster relief.

For the farmer, Drona means convenience. Drones can be directed to monitor crop health, pest infestations, or irrigation levels, using simple, native language commands. The once complex process is now as straightforward as chatting with a friend.

![image](https://github.com/Gaurang-1402/Drona/assets/71042887/dc9a2f40-bdd5-4f6f-95a9-d5f4e8228007)


In disaster relief, time is of the essence. Through Drona, relief workers can promptly deploy drones, making them search inaccessible areas, deliver vital supplies, or establish communication. With multilingual support, on-the-ground collaboration becomes easy, irrespective of geography.

![image](https://github.com/Gaurang-1402/Drona/assets/71042887/bd6a2b2f-eee1-4592-b031-e4c3ef1fca39)

## How we built it ⚙️

![image](https://github.com/Gaurang-1402/Drona/assets/71042887/9fa39a1f-b3a0-47d5-b095-6db7e826a476)


Our system seamlessly integrates a blend of cutting-edge technologies and frameworks to deliver streamlined drone operations and enhanced interactions:

"rosgpt.py": This is the central component of our system. Embedded within an ROS2-Humble node, it functions as a REST server that welcomes instructions from users through POST requests. With the immense capabilities of LangChain and its tool, CustomCommandToJSON(), it translates these instructions into structured JSON commands. Once processed, these commands are broadcasted on the /voice_cmd topic, ready for the next stage.

"rosgpt_client_node.py": Operating as an ROS2 client node, this component acts as the vital link between the user and the "rosgpt_translator_server.py". It directs POST requests with the user's commands to our REST server. When LangChain finishes processing, the converted JSON commands are presented back, providing immediate feedback to the user.

"rosgpt_client.py": Parallel in many ways to "rosgpt_client_node.py", its unique feature is its standalone capability. It predominantly operates as a REST client for the LangChain-integrated translator, free from ROS2 node engagement.

"rosgpt_parser_drone.py": Integrated with Gazebo for immersive drone simulation, this script springs into action post the translation phase. Termed as ROSGPTParser, its main job is to enforce the received commands. It latches onto the /voice_cmd topic, absorbs the JSON commands, and interprets them with tools like RetrievePOICoordinates() and ComputeDroneMovements(). Following the parsing, the necessary drone activities are initiated.

To further enhance the user experience, we developed an intuitive web application using Next.js. This interface is not just for issuing commands but is also deeply intertwined with LangChain capabilities such as PhraseClarifyingQuestion(), making user interaction more refined and intuitive. Through this collaboration, our autonomous agents can dynamically deduce and take actions, all while ensuring safety through tools like CheckCommandIsSafe().

Delving into LangChain:
Central to our architecture is LangChain, an innovative framework designed to spawn autonomous agents leveraging the might of large language models (LLMs). The capability of these agents to decide and act, combined with the textual proficiency of LLMs, sets LangChain as a prospective paradigm shift in multifarious AI use cases.

From crafting custom content, facilitating instantaneous translations, to bringing forth knowledgeable virtual assistants, LangChain emerges as the frontier of AI advancements. Even though it's evolving, the synergy it exhibits with LLMs is nudging the frontiers of what's achievable with AI across varied industries.

Melding ROS2-Humble, Gazebo, Next.js, and most critically, LangChain, our ROSGPT system breaks conventional norms, presenting a futuristic solution in drone operation and interactivity.


## Team 🫂

![image](https://github.com/Gaurang-1402/Drona/assets/71042887/a8114450-6ed0-4bc5-b477-460d0ed36cbb)


## Credits 📃

Simulation adapted from: https://github.com/NovoG93/sjtu_drone

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
