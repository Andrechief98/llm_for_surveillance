# LLM for Surveillance

This repository contains the software for an LLM-driven indoor surveillance system integrated with ROS, Gazebo, and TurtleBot3 robots. It includes the ROS packages, LLM interfaces, and supporting utilities used by the surveillance application.

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
- `Dockerfile`: Instructions for building the development environment image.
- `requirements.txt`: Python dependencies used by the application.


## Requirements
- Docker Desktop with Linux containers enabled
- An OpenAI API key for LLM functionality


## Run with Docker
### 1. Clone the repository

Clone the `Andrea` branch:

```bash
git clone -b Andrea https://github.com/Andrechief98/llm_for_surveillance.git
cd llm_for_surveillance
```

### 2. Build the image
Build the Docker image from the included `Dockerfile`:

```bash
docker build --no-cache -t llm-surveillance:latest .
```

### 3. Enable GUI access
Allow Docker containers to access the host's X11 display:
```bash
xhost +local:docker
``` 

### 4. Run the container
Start a new container and open an interactive Bash shell:
```bash
docker compose run --rm llm_surveillance_container 
```
The container is automatically removed when you exit the shell.


## Evaluation
Navigate to llm_data_analysis folder and run Streamlit app
```bash
cd /home/ros_ws/src/llm_for_surveillance/llm_data_analysis/spatial_reasoning && streamlit run spatial_reasoning_evaluation_app.py 
```
Open a new page in the browser to the link:

[http://localhost:8501](http://localhost:8501)


## Simulation

### 1. Launching Gazebo
Inside the container, run:
```bash
roslaunch utilities demo_multi_robot.launch
```

### 2. Start the Chainlit application (LLM interface)
Open a new terminal in the container:
```bash
cd /llm_for_surveillance/llm_interface/src
chainlit run surveillance_supervisor_app.py --host 0.0.0.0 --port 8000
```
Once the application starts, open the following address in your browser:

[http://localhost:8000](http://localhost:8000)
