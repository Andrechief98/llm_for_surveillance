
# !/bin/bash

# # Parameters
# world_id="1"
# llm_modality="rag_with_functions"         # HITL - AUTO - rag_with_functions - rag_with_semantics - image - image_and_rag_with_functions - image_and_rag_with_semantics 
# semantic_RAG="false"
# teleport_robots="true"
# print_rag_search="false"
# trial_id="1"

# sensor_sequence=(
#     "Lidar_4" 
#     "Lidar_3"
#     "Camera_1" 
#     "Lidar_2"
# )

# robot_list=(
#     "turtlebot3_1"
#     "turtlebot3_2"
# )

# # Conversione della sequenza in una stringa
# sensor_sequence_str=$(IFS=,; echo "${sensor_sequence[*]}")

# # Conversione della lista dei robot in una stringa
# robot_list_str=$(IFS=,; echo "${robot_list[*]}")

# # Avvia roslaunch in un terminale separato
# gnome-terminal --title="ROS Experiment" -- bash -c "
# roslaunch utilities demo_multi_robot.launch \
#     world_id:='$world_id' \
#     llm_modality:='$llm_modality' \
#     semantic_RAG:='$semantic_RAG' \
#     teleport_robots:='$teleport_robots' \
#     print_rag_search:='$print_rag_search' \
#     sensor_sequence:='$sensor_sequence_str'\
#     trial_id:='$trial_id';
# exec bash
# "
# sleep 5

# gnome-terminal --title="LLM chainlit" -- bash -c '
# cd ~/ros_packages_aggiuntivi/src/llm_for_surveillance/llm_interface/src || exit 1
# chainlit run prova_interfaccia_ros.py -w
# exec bash
# '

# sleep 20

# for stringa in "${sensor_sequence[@]}"; do
    
#     # Verifica se la stringa è un robot
#     found=false

#     for robot in "${robot_list[@]}"; do
#         if [[ "$stringa" == "$robot" ]]; then
#             found=true
#             break 
#         fi
#     done

#     if [[ "$found" == true ]]; then
#         # Call the ros service to deactivate the sensor
#         echo "$stringa deactivated "
#         rosservice call /deactivate_robot "data: '$stringa'"
#         # break 
        
#     else
#         # Call the ros service to activate the sensor 
#         echo "$stringa activated"
#         rosservice call /activateSensor "data: '$stringa'"

#     fi
#     # Pausa per poter far arrivare il current state all'LLM
#     sleep 10
# done


echo "🚀 Avvio di Chainlit..."
chainlit run app.py -h &
CHAINLIT_PID=$!
sleep 2 # Aspetta che il server sia pronto

echo "🖥️ Apertura finestra di verifica Chrome..."
google-chrome --app=http://localhost:8000 --user-data-dir=/tmp/chainlit_chrome_profile &
CHROME_PID=$!

echo "Premere [INVIO] nel terminale quando hai finito la verifica per chiudere tutto..."
read

echo "🧹 Chiusura in corso..."
kill $CHROME_PID
kill $CHAINLIT_PID

echo "✅ Fatto!"