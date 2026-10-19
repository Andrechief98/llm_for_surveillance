#!/bin/bash

# PARAMETERS

world_id="1"
trial_id="1"

llm_modality="rag_with_functions"
semantic_RAG="false"
teleport_robots="true"
print_rag_search="false"

SENSOR_PARAM="sensor_sequence"

CHAINLIT_DIR="$HOME/ros_packages_aggiuntivi/src/llm_for_surveillance/llm_interface/src"

robot_list=(
    "turtlebot3_1"
    "turtlebot3_2"
)

TERMINAL_PIDS=()

# SENSORS SEQUENCE FOR EACH WORLD

declare -A world_sequences

world_sequences["1"]="Lidar_3;Lidar_5,Camera_2;"
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
    semantic_RAG:="$semantic_RAG" \
    teleport_robots:="$teleport_robots" \
    print_rag_search:="$print_rag_search" \
    trial_id:="$trial_id" &

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

    # START CHAINLIT TERMINAL

    CHAINLIT_PID_FILE="/tmp/chainlit_${USER}_$$.pid"

    gnome-terminal --title="LLM Chainlit" -- bash -c "
        cd '$CHAINLIT_DIR' || exit 1

        # Avvia Chainlit e registra il PID reale
        chainlit run prova_interfaccia_ros.py -w &
        CHAINLIT_PID=\$!

        echo \$CHAINLIT_PID > '$CHAINLIT_PID_FILE'

        wait \$CHAINLIT_PID
    " &

    # Wait 10 seconds for Chainlit
    sleep 10

    # ACTIVATE SENSORS / DEACTIVATE ROBOTS

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
        echo "Attendo il completamento generazione risposta ..."

        timeout_seconds=180
        elapsed=0
        while true; do
            done_status=$(rosparam get /chainlit_done 2>/dev/null || echo "false")

            if [[ "$done_status" == "true" ]]; then
                echo "Generazione e salvataggio completati."
                break
            fi

            if (( elapsed >= timeout_seconds )); then
                echo "ERRORE: timeout durante l'attesa di Chainlit."
                break
            fi

            sleep 1
            elapsed=$((elapsed + 1))
        done
        
    done

    # CLOSE CHAINLIT

    echo "Chiusura Chainlit..."

    if [[ -f "$CHAINLIT_PID_FILE" ]]; then
        CHAINLIT_PID=$(cat "$CHAINLIT_PID_FILE")

        if kill -0 "$CHAINLIT_PID" 2>/dev/null; then
            kill -TERM "$CHAINLIT_PID" 2>/dev/null

            # Attende al massimo 10 secondi
            for ((i=0; i<10; i++)); do
                if ! kill -0 "$CHAINLIT_PID" 2>/dev/null; then
                    break
                fi
                sleep 1
            done

            # Forza la chiusura se ancora attivo
            if kill -0 "$CHAINLIT_PID" 2>/dev/null; then
                kill -KILL "$CHAINLIT_PID" 2>/dev/null
            fi
        fi

        rm -f "$CHAINLIT_PID_FILE"
    fi

    echo "Chainlit terminato."

    # RESET SENSORS

    echo "Reset sensori..."
    rosservice call /resetSensorActivation "{}"

    echo "Sequence $sequence_number completed."

done

# CLOSE ROS

echo "Chiusura simulazione ROS..."
kill "$ROS_PID" 2>/dev/null

echo "Tutte le sequenze sono state completate."