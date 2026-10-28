#!/bin/bash

# PARAMETERS

world_id="2"
trial_id="3"

llm_modality="image_and_rag_with_semantics"
teleport_robots="true"
print_rag_search="false"
llm_model="gpt-6-luna"

SENSOR_PARAM="sensor_sequence"

CHAINLIT_DIR="$HOME/ros_packages_aggiuntivi/src/llm_for_surveillance/llm_interface/src"

robot_list=(
    "turtlebot3_1"
    "turtlebot3_2"
)

TERMINAL_PIDS=()

# SENSORS SEQUENCE FOR EACH WORLD

declare -A world_sequences

world_sequences["1"]="Lidar_3;Lidar_5,Camera_2;Lidar_3,Camera_1,Lidar_2;Camera_2,Lidar_6,Lidar_7,Lidar_8" 
world_sequences["2"]="Camera_9;Lidar_14,Lidar_9;Camera_10,Lidar_2,Lidar_3;Lidar_10,Lidar_2,Lidar_1,Camera_5"

# SELECT WORLD

if [[ ! -v world_sequences["$world_id"] ]]; then
    echo "Errore: nessuna sequenza definita per world_id=$world_id"
    exit 1
fi

IFS=';' read -r -a sequences <<< "${world_sequences[$world_id]}"

# START ROS ONCE

echo "Avvio simulazione ROS..."

roslaunch utilities demo_multi_robot.launch \
    world_id:="$world_id" \
    llm_modality:="$llm_modality" \
    teleport_robots:="$teleport_robots" \
    print_rag_search:="$print_rag_search" \
    trial_id:="$trial_id" \
    llm_model:="$llm_model" &

ROS_PID=$!

# Wait 20 seconds for ROS
sleep 20

# LOOP ON SEQUENCES

sequence_number=0

for sequence_str in "${sequences[@]}"; do

    [[ -z "$sequence_str" ]] && continue

    sequence_number=$((sequence_number + 1))

    IFS=',' read -r -a sensor_sequence <<< "$sequence_str"
    sensor_sequence_str=$(IFS=,; echo "${sensor_sequence[*]}")

    echo
    echo "========================================"
    echo "World: $world_id"
    echo "Trial: $trial_id"
    echo "Sequence: $sequence_number"
    echo "Sensors: $sensor_sequence_str"
    echo "========================================"

    # UPDATE ROS PARAMETER

    rosparam set "$SENSOR_PARAM" "$sensor_sequence_str"

    sleep 2

    CHAINLIT_PID_FILE="/tmp/chainlit_${USER}_$$.pid"
    CHROME_PID_FILE="/tmp/chrome_${USER}_$$.pid"

    # --- START CHAINLIT & CHROME ---

    gnome-terminal --title="LLM Chainlit" -- bash -c "
        cd '$CHAINLIT_DIR' || exit 1

        # Avvia Chainlit
        chainlit run prova_interfaccia_ros.py --headless &
        CHAINLIT_PID=\$!
        echo \$CHAINLIT_PID > '$CHAINLIT_PID_FILE'

        # Aspetta che l'interfaccia sia pronta
        sleep 2

        # Avvia Chrome in una nuova finestra dedicata e salva il suo PID
        google-chrome --new-window http://localhost:8000 &
        CHROME_PID=\$!
        echo \$CHROME_PID > '$CHROME_PID_FILE'

        # Rimane in ascolto del processo Chainlit
        wait \$CHAINLIT_PID
    " &

    # Wait 10 seconds for Chainlit
    sleep 10

    # ACTIVATE SENSORS / DEACTIVATE ROBOTS
    rosparam set /valid_run true
    
    for i in "${!sensor_sequence[@]}"; do

        rosparam set /chainlit_done false

        stringa="${sensor_sequence[$i]}"
        found=false

        for robot in "${robot_list[@]}"; do
            if [[ "$stringa" == "$robot" ]]; then
                found=true
                break
            fi
        done

        if [[ "$found" == true ]]; then
            echo "$stringa deactivated"
            rosservice call /deactivate_robot "data: '$stringa'"
        else
            echo "$stringa activated"
            rosservice call /activateSensor "data: '$stringa'"
        fi

        # Wait untile the LLM finishes to generate the response (or wait 60 seconds)
        echo "Waiting response generation completion ..."

        timeout_seconds=180
        elapsed=0
        while true; do
            done_status=$(rosparam get /chainlit_done 2>/dev/null || echo "false")

            if [[ "$done_status" == "true" ]]; then
                echo "Generation completed."
                break
            fi

            if (( elapsed >= timeout_seconds )); then
                echo "ERRORE: reached Chainlit timeout."
                rosparam set /valid_run false
                break
            fi

            sleep 1
            elapsed=$((elapsed + 1))
        done
        
    done

    # --- CHROME ---
    echo "Shutting down Chrome..."

    if [[ -f "$CHROME_PID_FILE" ]]; then
        CHROME_PID=$(cat "$CHROME_PID_FILE")

        if kill -0 "$CHROME_PID" 2>/dev/null; then
            # Tenta una chiusura soft (invia il segnale di chiusura alla finestra)
            kill -TERM "$CHROME_PID" 2>/dev/null

            # Attende che si chiuda (max 5 secondi)
            for ((i=0; i<5; i++)); do
                if ! kill -0 "$CHROME_PID" 2>/dev/null; then
                    break
                fi
                sleep 1
            done

            # Forza la chiusura se la finestra è bloccata
            if kill -0 "$CHROME_PID" 2>/dev/null; then
                kill -KILL "$CHROME_PID" 2>/dev/null
            fi
        fi
        rm -f "$CHROME_PID_FILE"
    fi


    # --- CHAINLIT ---
    echo "Shutting down Chainlit..."

    if [[ -f "$CHAINLIT_PID_FILE" ]]; then
        CHAINLIT_PID=$(cat "$CHAINLIT_PID_FILE")

        if kill -0 "$CHAINLIT_PID" 2>/dev/null; then
            kill -TERM "$CHAINLIT_PID" 2>/dev/null

            for ((i=0; i<10; i++)); do
                if ! kill -0 "$CHAINLIT_PID" 2>/dev/null; then
                    break
                fi
                sleep 1
            done

            if kill -0 "$CHAINLIT_PID" 2>/dev/null; then
                kill -KILL "$CHAINLIT_PID" 2>/dev/null
            fi
        fi
        rm -f "$CHAINLIT_PID_FILE"
    fi

    # RESET SENSORS

    echo "Sensors reset ..."
    rosservice call /resetSensorActivation "{}"

    echo "Sequence $sequence_number completed."

    sleep 5

done

# CLOSE ROS

if kill -0 "$ROS_PID" 2>/dev/null; then
    kill -INT "$ROS_PID" 2>/dev/null
    wait "$ROS_PID" 2>/dev/null
fi

echo "Tutte le sequenze sono state completate."