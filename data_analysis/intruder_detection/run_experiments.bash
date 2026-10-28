#!/bin/bash

# PARAMETERS
experiment="deterministic"
world_id="2"
trial_id="3"

modality="HITL"
teleport_robots="true"
print_rag_search="false"
llm_model="gpt-6-luna"

robot_list=(
    "turtlebot3_1"
    "turtlebot3_2"
)

SENSOR_PARAM="sensor_sequence"

# PATHS
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

CHAINLIT_DIR="$PROJECT_DIR/../../llm_interface/src"
HUMAN_DIR="$PROJECT_DIR/../../human_supervisory_interface/src"

# PYTHON SCRIPTS
# - for HITL e AUTO
LLM_SCRIPT="llm_ros_interface.py"

# - for HUMAN
HUMAN_SCRIPT="supervisory_ros_interface.py"

# Verify that the application directories exist.
if [[ ! -d "$CHAINLIT_DIR" ]]; then
    echo "ERROR: Chainlit directory not found: $CHAINLIT_DIR"
    exit 1
fi

if [[ ! -d "$HUMAN_DIR" ]]; then
    echo "ERROR: Human interface directory not found: $HUMAN_DIR"
    exit 1
fi

case "$modality" in
    HITL|AUTO)
        PYTHON_DIR="$CHAINLIT_DIR"
        PYTHON_SCRIPT="$LLM_SCRIPT"
        USE_CHAINLIT=true
        USE_CHROME=true
        ;;

    HUMAN)
        PYTHON_DIR="$HUMAN_DIR"
        PYTHON_SCRIPT="$HUMAN_SCRIPT"
        USE_CHAINLIT=false
        USE_CHROME=false
        ;;

    *)
        echo "Error: modality not valid: $modality"
        exit 1
        ;;
esac

if [[ ! -f "$PYTHON_DIR/$PYTHON_SCRIPT" ]]; then
    echo "Error: file not found: $PYTHON_DIR/$PYTHON_SCRIPT"
    exit 1
fi



# SENSORS SEQUENCE FOR EACH WORLD AND MODALITY
# Each configuration is identified by experiment type and world ID.
#
# Record format:
# sensor1,sensor2,...|target
#
# Multiple sensors are separated by commas.
# The target represents the expected final intruder position.
# RAND(A,B,C) indicates that the target must be randomly selected
# from the specified set of possible positions.

declare -A experiment_sequences

# ==================================================
# WORLD 1
# ==================================================

# DETERMINISTIC - 
# Format: sensor sequence | target area
experiment_sequences["deterministic,1"]=$(cat <<'EOF'
Camera_1|B
Lidar_3|C
Lidar_6|G
Lidar_2|A
Lidar_5,Camera_2|F
Lidar_6,Lidar_1|A
Lidar_5,Lidar_4|C
Lidar_7,Lidar_6|E
Lidar_3,Camera_1,Lidar_2|A
Lidar_1,Lidar_5,Lidar_4|C
Camera_2,Lidar_6,Lidar_7|G
Lidar_6,Lidar_7,Lidar_8|I
Camera_2,Lidar_6,Lidar_7,Lidar_8|L
Lidar_7,Lidar_6,Lidar_5,Lidar_4|C
Lidar_5,Lidar_6,Lidar_1,Lidar_2|A
Lidar_1,Lidar_8,Lidar_7,Lidar_6|E
EOF
)

# UNCERTAIN 
# Random target with RAND(...)
experiment_sequences["uncertain,1"]=$(cat <<'EOF'
Camera_2|-
Lidar_8|RAND(L,A,I)
Lidar_1|-
Lidar_5|RAND(F,C,D)
Camera_1,Lidar_3|RAND(E,A,C)
Lidar_1,Lidar_8|RAND(H,G,B)
Lidar_3,Lidar_7|C
Lidar_4,Lidar_1|E
Camera_1,Lidar_6,Lidar_5|G
Lidar_8,Lidar_1,Camera_2|A
Lidar_4,Lidar_5,Lidar_6|RAND(B,I,F)
Lidar_2,Lidar_1,Lidar_8|RAND(G,E,B)
Camera_1,Lidar_2,Lidar_1,Lidar_7|A
Lidar_8,Lidar_1,Camera_1,Lidar_3|C
Lidar_5,Lidar_4,Lidar_3,Camera_1|RAND(E,I,A)
Lidar_3,Camera_1,Lidar_6,Lidar_7|RAND(L,E,I)
EOF
)

# ==================================================
# WORLD 2
# ==================================================

# DETERMINISTIC - 
# Format: sensor sequence | target area
experiment_sequences["deterministic,2"]=$(cat <<'EOF'
Camera_9|B
Lidar_14,Lidar_9|C
Camera_10,Lidar_2,Lidar_3|A
EOF
)

# UNCERTAIN 
# Random target with RAND(...)
experiment_sequences["uncertain,2"]=$(cat <<'EOF'
Camera_9|B
Lidar_14,Lidar_9|C
Camera_10,Lidar_2,Lidar_3|A
EOF
)



# SELECT WORLD, EXPERIMENT AND SEQUENCE
# Retrieve the configuration matching the experiment type and world ID.
# Stop execution if the requested configuration does not exist.

config_key="${experiment},${world_id}"

if [[ ! -v experiment_sequences["$config_key"] ]]; then
    echo "Error: no sequence defined for:"
    echo "  experiment=$experiment"
    echo "  world_id=$world_id"
    exit 1
fi

# Load all configured sequences into a Bash array.
mapfile -t sequences < <(
    printf '%s\n' "${experiment_sequences[$config_key]}"
)

# Randomize the order of the complete records.
# Each record is shuffled as a whole, preserving the association
# between its sensor list and its target.
mapfile -t randomized_sequences < <(
    printf '%s\n' "${sequences[@]}" | shuf
)


# START ROS ONCE

echo "Launching ROS simulation..."

roslaunch utilities demo_multi_robot.launch \
    world_id:="$world_id" \
    modality:="$modality" \
    teleport_robots:="$teleport_robots" \
    print_rag_search:="$print_rag_search" \
    trial_id:="$trial_id" \
    llm_model:="$llm_model" &

ROS_PID=$!

# Wait 20 seconds for ROS
sleep 20

# EXECUTE THE RANDOMIZED SENSOR SEQUENCES
# Each configured record is processed exactly once per trial.

sequence_number=0

for record in "${randomized_sequences[@]}"; do

    # Skip empty records.
    [[ -z "$record" ]] && continue

    sequence_number=$((sequence_number + 1))

    # Split the record into the sensor list and the target specification.
    IFS='|' read -r sensors_str target_spec <<< "$record"

    # Convert the comma-separated sensor list into a Bash array.
    IFS=',' read -r -a sensor_sequence <<< "$sensors_str"

    # RESOLVE THE TARGET POSITION
    # If the target uses RAND(...), randomly select one of the
    # available target positions.
    # Otherwise, retain the predefined target.

    if [[ "$target_spec" == RAND\(*\) ]]; then

        # Extract the list of candidate targets from RAND(...).
        choices="${target_spec#RAND(}"
        choices="${choices%)}"

        # Convert the candidate targets into a Bash array.
        IFS=',' read -r -a target_options <<< "$choices"

        # Select one candidate target at random.
        target_spec="${target_options[RANDOM % ${#target_options[@]}]}"

    fi

    echo "========================================"
    echo "Experiment: $experiment"
    echo "World ID: $world_id"
    echo "Trial ID: $trial_id"
    echo "Sequence: $sequence_number"
    echo "========================================"

    # Update the ROS parameter with the current sensor sequence.
    rosparam set "$SENSOR_PARAM" "$sensors_str"

    # Publish the expected final intruder position.
    rosparam set /expected_intruder_position "$target_spec"


    sleep 2

    CHAINLIT_PID_FILE="/tmp/chainlit_${USER}_$$.pid"
    CHROME_PID_FILE="/tmp/chrome_${USER}_$$.pid"


    # --- START CHAINLIT/STREAMLIT & CHROME ---

    gnome-terminal --title="$modality Interface" -- bash -c "
        cd '$PYTHON_DIR' || exit 1

        if [[ '$USE_CHAINLIT' == 'true' ]]; then
            chainlit run '$PYTHON_SCRIPT' --headless &
        else
            streamlit run '$PYTHON_SCRIPT' &
        fi

        PYTHON_PID=\$!
        echo \$PYTHON_PID > '$PYTHON_PID_FILE'

        if [[ '$USE_CHROME' == 'true' ]]; then
            sleep 2
            google-chrome --new-window http://localhost:8000 &
            CHROME_PID=\$!
            echo \$CHROME_PID > '$CHROME_PID_FILE'
        fi

        wait \$PYTHON_PID
    " &

    # Wait 10 seconds for Chainlit/Streamlit
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