# llm_for_surveillance

## Requirements
- ROS1 Noetic

## How to use it

Launch the simulation:
```bash
    roslaunch tim_utilities timDemo_multi_robot.launch
```
Launch the PS4 controller node for actor teleoperation
```bash
    roslaunch tim_utilities timDemo_teleop.launch
```
Run the main script `app_ros.py` to launch the chat on the host (http://127.0.0.1:5000/) and interact with the LLM via API 
