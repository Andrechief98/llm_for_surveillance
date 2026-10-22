# ROS Noetic environment for the LLM-driven surveillance project
FROM osrf/ros:noetic-desktop-full

ENV CATKIN_WS=/home/ros_ws 

# Sourcing ros
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc



# Install system and ROS dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-pip \
    curl \
    nano \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-msgs \
    ros-noetic-navigation \
    ros-noetic-cob-srvs \
    ros-noetic-visualization-msgs \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*


# Updating rosdep
RUN rosdep init && rosdep update

# Going in the workspace
WORKDIR ${CATKIN_WS}/src

# Cloning actor plugin repository
RUN git clone -b noetic-classic https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git

# Adding folders from current repository
COPY llm_interface ./llm_for_surveillance/llm_interface
COPY utilities ./llm_for_surveillance/utilities
COPY llm_data_analysis ./llm_for_surveillance/llm_data_analysis
COPY requirements.txt ./llm_for_surveillance/requirements.txt

# Include the additional plugins required by sensors and actuators in the simulation
COPY gazebo_plugins ./gazebo_plugins

# Install Python dependencies
WORKDIR ${CATKIN_WS}/src/llm_for_surveillance
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# Build the Catkin workspace
WORKDIR ${CATKIN_WS}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Sourcing the ros workspace
RUN echo "source /home/ros_ws/devel/setup.bash" >> /root/.bashrc

# Disable ROS Noetic EOL
RUN echo "export DISABLE_ROS1_EOL_WARNINGS="true"" >> /root/.bashrc

# Setting gazebo plugin directory
RUN echo "export GAZEBO_PLUGIN_PATH=\${GAZEBO_PLUGIN_PATH}:${CATKIN_WS}/devel/lib" >> /root/.bashrc


# Default directory for the surveillance application
WORKDIR ${CATKIN_WS}/src/llm_for_surveillance

# Exposing Chainlit port (8000) and Streamlit port (8501)
EXPOSE 8000 
EXPOSE 8501

CMD ["bash"]