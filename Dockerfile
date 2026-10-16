# ROS Noetic environment for the LLM-driven surveillance project
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV CATKIN_WS=/root/catkin_ws

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install system and ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-catkin-tools build-essential ros-noetic-gazebo-ros ros-noetic-gazebo-msgs ros-noetic-turtlebot3 ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-gazebo && rm -rf /var/lib/apt/lists/*
RUN pip install streamline

# Prepare the Catkin workspace and copy the project
WORKDIR ${CATKIN_WS}/src

COPY llm_interface ./llm_for_surveillance/llm_interface
COPY utilities ./llm_for_surveillance/utilities
COPY llm_data_analysis ./llm_for_surveillance/llm_data_analysis
COPY requirements.txt ./llm_for_surveillance/requirements.txt
COPY README.md ./llm_for_surveillance/README.md

# Include the additional ROS packages required by the workspace
COPY extra_ros_packages/gazebo_plugins ./gazebo_plugins
COPY extra_ros_packages/llm_robot ./llm_robot

# Install Python dependencies
WORKDIR ${CATKIN_WS}/src/llm_for_surveillance
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# Build the Catkin workspace
WORKDIR ${CATKIN_WS}
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Default directory for the surveillance application
WORKDIR ${CATKIN_WS}/src/llm_for_surveillance/llm_interface/src

EXPOSE 8000

CMD ["bash"]