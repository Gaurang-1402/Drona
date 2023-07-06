# ChatDrones: Control Drones with Natural Language

ChatDrones is a project that integrates Language Logic Models (LLMs) with drone control. This setup allows users to **command drones using simple, everyday language.** The project uses the ROS2 (Robot Operating System) Humble and runs simulations in the Gazebo environment, providing a reliable platform for drone behavior.

ChatDrones also includes a web application with a user-friendly interface, making it easy for users to input commands. This combines the benefits of advanced technology with a simple and easy-to-use design.

Currently, ChatDrones operates within a simulated environment, serving as a practical platform for testing and development. However, it's also designed to work with real drones, translating natural language instructions into actual drone commands.

Key features include:

- A user-friendly web application that provides an interactive interface for drone control
- Ability to initiate drone landing and takeoff through simple natural language commands
[![output-onlinegiftools-14-1.gif](https://i.postimg.cc/rsQSKyS4/output-onlinegiftools-14-1.gif)](https://postimg.cc/hQQJYBZ4)
- Full control over the drone's directional movement, including forward, backward, left, and right commands
[![output-onlinegiftools-16-1.gif](https://i.postimg.cc/KjHR61P0/output-onlinegiftools-16-1.gif)](https://postimg.cc/64CWR51Z)
- Support for commands in multiple languages

[![output-onlinegiftools-17-1.gif](https://i.postimg.cc/qvXpntNW/output-onlinegiftools-17-1.gif)](https://postimg.cc/mPkxWgNS)



## ROSGPT Architecture

![Screenshot from 2023-07-04 20-49-50](https://github.com/Gaurang-1402/ChatDrones/assets/71042887/f3534fd5-1ac8-4d55-8e67-fb5f6c0ddf8d)


## Getting started

Clone the repository

```
mkdir ros_ws
cd ros_ws
git clone <repo_url>
```

Install rosgpt libraries from the rosgpt folder

```
cd ~/src/rosgpt
pip3 install -r requirements.txt
```

Install ROS requirements

```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

```
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=<rosdistro> -y
```


Add your OpenAI API Key in your ```.bashrc``` as an environment variable 

```
echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```


## Running ROSGPT

First, navigate to the root directory of your workspace and build the project

```
cd ~/ros_ws
colcon build --symlink-install
```
Now run each of these commands on new terminals

```
source install/setup.sh
ros2 run rosgpt rosgpt
```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```

```
source install/setup.sh
ros2 run rosgpt rosgptparser_drone 
```

## Running the simulation

```
source install/setup.sh
ros2 launch  sjtu_drone_bringup sjtu_drone_bringup.launch.py
```


Note: Please replace `<repository_url>` and `<your_api_key>` with the actual repository URL and your OpenAI API key, respectively.


## Credits
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

I am deeply appreciative of these individuals/teams for sharing their work to build on top of!
