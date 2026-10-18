
#!/bin/bash

sensor_sequence=(
    "Lidar_1" 
    "Lidar_2" 
    "turtlebot3_1"
)

robot_list=(
    "turtlebot3_1"
    "turtlebot3_2"
)

# Parameters
world_id="1"
llm_modality="HITL"
semantic_RAG="false"
teleport_robots="true"
print_rag_search="false"

# Conversione della sequenza in una stringa
sensor_sequence_str=$(IFS=,; echo "${sensor_sequence[*]}")

# Conversione della lista dei robot in una stringa
robot_list_str=$(IFS=,; echo "${robot_list[*]}")

# Avvia roslaunch in un terminale separato
gnome-terminal --title="ROS Experiment" -- bash -c "
roslaunch utilities demo_multi_robot.launch \
    world_id:='$world_id' \
    llm_modality:='$llm_modality' \
    semantic_RAG:='$semantic_RAG' \
    teleport_robots:='$teleport_robots' \
    print_rag_search:='$print_rag_search' \
    sensor_sequence:='$sensor_sequence_str';
exec bash
"

sleep 20

for stringa in "${sensor_sequence[@]}"; do
    
    # Verifica se la stringa è un robot
    found=false

    for robot in "${robot_list[@]}"; do
        if [[ "$stringa" == "$robot" ]]; then
            found=true
            break 
        fi
    done

    if [[ "$found" == true ]]; then
        # Call the ros service to deactivate the sensor
        echo "$stringa deactivated "
        rosservice call /deactivate_robot "data: '$stringa'"
        # break 
        
    else
        # Call the ros service to activate the sensor 
        echo "$stringa activated"
        rosservice call /activateSensor "data: '$stringa'"

    fi
    # Pausa per poter far arrivare il current state all'LLM
    sleep 3
done
