# LLM for Surveillance

This repository contains the software for an LLM-driven indoor surveillance system integrated with ROS, Gazebo, and TurtleBot3 robots. It includes the ROS packages, LLM interfaces, and supporting utilities used by the surveillance application.

## Requirements

### Option 1: Docker (Recommended)

- Docker Desktop with Linux containers enabled
- An OpenAI API key for LLM functionality

Docker provides a reproducible environment with ROS Noetic, the required system packages, Python dependencies, and the Catkin workspace.

### Option 2: Native installation

For running the project directly on Ubuntu, the main requirements are:

- Ubuntu 20.04 LTS
- ROS 1 Noetic
- TurtleBot3 ROS packages and Gazebo integration
- Python 3 and the packages listed in `requirements.txt`

Refer to the [ROS Noetic documentation](https://wiki.ros.org/noetic) and the [TurtleBot3 documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) for installation instructions.

## Run with Docker

### 1. Clone the repository

Clone the `Ehsan` branch:

```bash
git clone -b Ehsan https://github.com/Andrechief98/llm_for_surveillance.git
cd llm_for_surveillance
```

### 2. Build the image

Build the Docker image from the included `Dockerfile`:

```bash
docker build -t llm-surveillance:dev .
```

The build installs the required ROS and system packages, copies the project and additional ROS packages into the Catkin workspace, installs the Python dependencies, and builds the workspace with `catkin_make`.

### 3. Configure the OpenAI API key

The surveillance application requires an active OpenAI API key for LLM functionality.

In Windows Command Prompt, set the key for the current terminal session:

```cmd
set OPENAI_API_KEY=YOUR_OPENAI_API_KEY
```

Replace `YOUR_OPENAI_API_KEY` with your own key. Do not commit API keys to Git or hardcode them in source files.

### 4. Start the container

Run:

```cmd
docker run -it --name llm-surveillance -p 8000:8000 -e OPENAI_API_KEY llm-surveillance:dev bash
```

This starts a container and opens a shell inside it.

### 5. Start the Chainlit application

Inside the container, run:

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
cd /root/catkin_ws/src/llm_for_surveillance/llm_interface/src
chainlit run surveillance_supervisor_app.py --host 0.0.0.0 --port 8000
```

Once the application starts, open the following address in your browser:

[http://localhost:8000](http://localhost:8000)

Keep the terminal running while using the application.

### 6. Start the mock TIM server (if required)

If the application needs the mock TIM backend, open a second terminal on the host and run:

```cmd
docker exec -it llm-surveillance bash
```

Inside the second container shell, run:

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
cd /root/catkin_ws/src/llm_for_surveillance/llm_interface/src
python3 mock_tim_server.py
```

Keep the mock server running while the application uses it.

### 7. Stop and restart the container

To stop the container:

```cmd
docker stop llm-surveillance
```

To start it again:

```cmd
docker start -ai llm-surveillance
```

If you need to remove the container and recreate it, first stop it and then run:

```cmd
docker rm llm-surveillance
```

The Docker image remains available locally unless you remove it separately.

## Running the ROS simulation

The Docker image builds the Catkin workspace, but starting the Chainlit interface alone does not launch the full ROS/Gazebo simulation.

Full surveillance functionality may require a running ROS master, Gazebo, the robot simulation, and the relevant sensor and control nodes.

The original simulation workflow is described below.

### 1. Build the workspace

For a native ROS installation, open a terminal in the Catkin workspace and run:

```bash
catkin_make
source devel/setup.bash
```

The Docker image already runs `catkin_make` during its build.

### 2. Prepare the OpenAI thread configuration

Before starting the application, check the file:

`llm_interface/src/Open AI threads.json`

If you need to reset the saved thread information, back up the file first and ensure its contents have the following structure:

```json
{
  "threads": []
}
```

Only reset this file when you intend to clear the stored thread references.

### 3. Launch the simulation

Launch the multi-robot simulation:

```bash
roslaunch utilities timDemo_multi_robot.launch
```

The PS4 controller launch command is available in the original project configuration but is commented out in the README:

```bash
roslaunch utilities timDemo_teleop.launch
```

### 4. Launch an LLM interface

The original ROS-integrated interfaces include:

- `llm_interface/src/app_ros_MITL.py`
- `llm_interface/src/app_ros_autonomous.py`

Run the appropriate script in an environment where ROS and the required Python dependencies are configured. For example:

```bash
python3 llm_interface/src/app_ros_MITL.py
```

Or:

```bash
python3 llm_interface/src/app_ros_autonomous.py
```

Follow the interface's startup output for the correct web address and port. The original README used `http://127.0.0.1:5000/` for these interfaces; the Docker-based Chainlit application described above uses port `8000`.

## Evaluate LLM responses
This section describes how to launch the LLM Conversation Reviewer, a web-based interface for inspecting LLM conversations and manually evaluating errors across different experimental modes, such as HITL and AUTO. The platform displays conversations in a human-readable format and saves evaluation results to a separate JSON file.

### 1. Go to the llm_data_analysis folder
Open a terminal and navigate to the project directory:

```bash
llm_data_analysis
```
### 2. Run the evaluation platform
Launch the Streamlit application:

```bash
streamlit run app_evaluation.py
```
The application will open in your default browser. If it does not open automatically, navigate to the URL displayed in the terminal (usually http://localhost:8501).

### 3. Load the conversation dataset
Upload the JSON file containing the experimental runs.
Select the experimental mode (HITL or AUTO).
Use the navigation arrows to move between runs.
Inspect the conversation, including user messages, assistant responses, function calls, and ROS outputs.

### 4. Evaluate the LLM responses
Define the error categories in the evaluation panel.
Assign the number of errors for each category.
Add notes to explain the evaluation, if necessary.
Click Save evaluation to save the results.

### 5. Retrieve the evaluation results
The platform saves the evaluation results in evaluations.json and the error categories in error_categories.json, in the directory from which Streamlit was launched.

The original conversation dataset is not modified.


## Repository structure

```text
llm_for_surveillance/
├── Dockerfile
├── .dockerignore
├── requirements.txt
├── llm_interface/
├── utilities/
├── llm_data_analysis/
└── extra_ros_packages/
    ├── gazebo_plugins/
    └── llm_robot/
```

- `llm_interface/`: LLM interfaces and application code.
- `utilities/`: ROS simulation, robot, and sensor utilities.
- `llm_data_analysis/`: Data analysis components.
- `extra_ros_packages/gazebo_plugins/`: Additional ROS package containing custom Gazebo-related services.
- `extra_ros_packages/llm_robot/`: Additional robot-related ROS package.
- `Dockerfile`: Instructions for building the development environment.
- `requirements.txt`: Python dependencies used by the application.

## Notes

- An OpenAI API key is required for LLM functionality.
- The Docker image builds the workspace, but external ROS services and Gazebo simulation may need to be started separately.
- The mock TIM server is intended for workflows that need a simulated TIM backend.
- The Docker setup and the original ROS-integrated applications use different startup commands and may expose different ports.