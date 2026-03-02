# llm_for_surveillance

## Requirements
- Ubuntu 20.04 LTS
- ROS1 Noetic - installation and invironment configuration guide in ROS wiki
- ROS plugins for the turtlebot3 (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- download in catkin worspace and built gazebo_plugins package
- python packages (install with `pip3 install`)
  - Flask v. 3.0.3
  - openai v. 1.99.9
  - werkzeug v. 3.0.6
  - PyYAML v. 5.3.1
  - python-socketio v. 5.15.0
  - flask-socketio v. 5.5.1
- download repo in your catkin workspace and build it


## How to use it

Before starting the simulation build the downloaded package with the `catkin_make` command and cancel the thread ID so that it acn be sobstituted with yours. to do this go in `.../llm_for_surveillance/llm_interface/src/Open AI threads.json` and cancel the data. It should look like this:
```json
{
  "threads": [

  ]
}
```

### Launch the simulation:

first launch the ROS simulation 
```bash
    roslaunch utilities timDemo_multi_robot.launch
```
<!-- Launch the PS4 controller node for actor teleoperation
```bash
    roslaunch utilities timDemo_teleop.launch
``` -->
The start the LLM, by running script

`../llm_for_surveillance/llm_interface/src/app_ros_MITL.py` 

or 

`.../llm_for_surveillance/llm_interface/src/app_ros_autonomous.py` 

using `/bin/python3 <sript>`

and launch chat in browser (http://127.0.0.1:5000/) to interact with the LLM.

Note that you need an active key to be able to use OpneAI LLM.

