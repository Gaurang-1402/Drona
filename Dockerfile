FROM osrf/ros:humble-desktop as base
ENV rosdistro=humble
ARG OPENAI_API_KEY
ENV OPENAI_API_KEY=$OPENAI_API_KEY
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update \
    && apt-get install -y \
    libespeak-dev \
    python3-pip \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    apt-utils 

# Create colcon workspace with dependencies
RUN mkdir /ros2_ws
COPY ./src /ros2_ws/src
COPY ./requirements.txt /ros2_ws/requirements.txt

# Install python dependencies
RUN pip3 install -r /ros2_ws/requirements.txt

# Set up rosdep
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --rosdistro $rosdistro --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install 

EXPOSE 5000

# Create a script to run the Flask app
RUN echo "#!/bin/bash" > /ros2_ws/start_flask_app.sh
RUN echo "source install/setup.sh" >> /ros2_ws/start_flask_app.sh
RUN echo "ros2 run rosgpt rosgpt &" >> /ros2_ws/start_flask_app.sh
RUN echo "ros2 run rosgpt rosgpt_client_node &" >> /ros2_ws/start_flask_app.sh
RUN echo "ros2 run rosgpt rosgptparser_drone &" >> /ros2_ws/start_flask_app.sh
RUN echo "ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py" >> /ros2_ws/start_flask_app.sh

RUN chmod +x /ros2_ws/start_flask_app.sh

ENTRYPOINT ["/ros2_ws/start_flask_app.sh"]
