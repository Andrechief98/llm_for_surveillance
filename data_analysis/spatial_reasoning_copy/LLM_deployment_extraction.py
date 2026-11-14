
import os
import json
from pathlib import Path
from openai import OpenAI


# ============================================================
# CONFIGURATION
# ============================================================

API_KEY = os.getenv("OPENAI_API_KEY")

if not API_KEY:
    raise ValueError("OPENAI_API_KEY not found.")

client = OpenAI(api_key=API_KEY)

INPUT_FILE = Path("experiments.json")

DEPLOYMENTS_FILE = Path("robot_deployments.json")
DISTANCES_FILE = Path("robot_area_distances.json")

MODEL = "gpt-6-astra"


# ============================================================
# EXPERIMENT SELECTION
# None = all available elements
# ============================================================

SELECTED_MODES = ["rag_with_semantics", "image", "image_and_rag_with_functions", "image_and_rag_with_semantics"]
# Example: ["rag_with_functions", "rag_with_semantics", "image","image_and_rag_with_functions", "image_and_rag_with_semantics"]

SELECTED_WORLDS = ["world_1", "world_2"]
# Example: ["world_1"]

SELECTED_MODELS = ["gpt-4o", "gpt-6-luna"]
# Example: [""]

SELECTED_TRIALS = ["trial_1", "trial_2", "trial_3"]
# Example: ["trial_1", "trial_3"]



# ============================================================
# ALLOWED ROBOTS AND AREAS
# ============================================================

ALLOWED_ROBOTS = [
    "turtlebot3_1",
    "turtlebot3_2"
]

WORLD_AREAS = {
    "world_1": [
        "A", "B", "C", "D", "E",
        "F", "G", "H", "I", "L"
    ],

    "world_2": [
        "A", "B", "C", "D", "E",
        "F", "G", "H", "I", "L",
        "M", "N", "O", "P", "Q",
        "R", "S", "T", "U", "V", "Z"
    ]
}


# ============================================================
# EXTRACT ASSISTANT MESSAGES
# ============================================================

def extract_assistant_messages(conversation: list) -> list:
    """
    Extracts only the values associated with the assistant key.

    Each assistant message represents one planning turn.
    The original conversation order is preserved.
    """

    return [
        item["assistant"]
        for item in conversation
        if isinstance(item, dict) and "assistant" in item
    ]


# ============================================================
# EXTRACT ALL HISTORICAL DISTANCES
# ============================================================

def extract_computed_distances(conversation: list) -> dict:
    """
    Extracts all historical distances from the conversation.

    Only ROS_output immediately following a
    compute_robot_area_distance function_call is considered.

    If an area is encountered multiple times, only its
    first complete result is retained.

    No distance calculation is performed.
    """

    computed_distance = {}

    for index, item in enumerate(conversation):

        if not isinstance(item, dict):
            continue

        if "function_call" not in item:
            continue

        function_call = item["function_call"]

        if not isinstance(function_call, str):
            continue

        if "compute_robot_area_distance" not in function_call:
            continue

        if index + 1 >= len(conversation):
            continue

        next_item = conversation[index + 1]

        if not isinstance(next_item, dict):
            continue

        if "ROS_output" not in next_item:
            continue

        try:
            distance_data = json.loads(
                next_item["ROS_output"]
            )

        except (json.JSONDecodeError, TypeError):

            print(
                f"Warning: invalid distance ROS_output "
                f"at index {index + 1}"
            )

            continue

        if not isinstance(distance_data, dict):
            continue

        for area, robot_distances in distance_data.items():

            if not isinstance(robot_distances, dict):
                continue

            if area in computed_distance:
                continue

            valid_distances = {}

            for robot, distance in robot_distances.items():

                if (
                    robot in ALLOWED_ROBOTS
                    and isinstance(distance, (int, float))
                    and not isinstance(distance, bool)
                ):
                    valid_distances[robot] = distance

            if valid_distances:
                computed_distance[area] = valid_distances

    return computed_distance


# ============================================================
# LLM: DEPLOYMENT EXTRACTION
# ============================================================

def extract_deployments(
    assistant_message: str,
    world_name: str
) -> list:

    if world_name not in WORLD_AREAS:
        raise ValueError(f"Unknown world: {world_name}")

    allowed_areas = WORLD_AREAS[world_name]

    system_prompt = """
    You are an expert assistant specialized in analyzing
    robotic inspection plans.
    
    Analyze ONLY the assistant message provided by the user.
    
    Identify the robot deployments explicitly proposed
    in this message.
    
    For every proposed deployment identify:
    
    - Robot name.
    - Target area.
    
    IMPORTANT RULES:
    
    1. Only consider explicit robot deployment proposals. For example in this case:
     "1. **Environmental Status:**\n   
     - **Activated Sensors:** Camera_2, Lidar_6, Lidar_7, Lidar_8\n   
     - **Areas:**\n     
     - **Involved areas:** F, G, H, I, L\n     
     - **Areas connections:**\n       
        - F is connected to E via an open passage.\n       
        - E is connected to G via door_6 (currently open).\n       
        - G is connected to H via door_7 (currently open).\n       
        - H is directly connected to I via an open passage.\n       
        - I is connected to L via door_8 (currently open).\n   
     - **Robots:**\n     
     - **Status:** Both available for deployment.\n     
     - **Location:** Both in area E.\n   
     - **Cameras:** \n     
        - **Camera_2** covers area F.\n\n2. 
     **Intruder Movement Hypotheses:**\n   
        - **Movements:** Sequential activation suggests movement from F to G, then G to H, then H or I to L. \n\n3. 
     **Proposed Inspection Plan:**\n\n   
        - **Areas to investigate:** F, G, H, L\n   
        - **Camera Inspections:** Use Camera_2 to inspect area F.\n   
        - **Robot Deployments:**\n     
            - Deploy **turtlebot3_2** to area F (shortest distance to assist camera inspection).\n     
            - Deploy **turtlebot3_1** to area G.\n     
            - After inspecting G, deploy **turtlebot3_1** to area H, then L as needed.\n\n4. 
    
     **Approval Request:**\n   
        - The plan has not been executed. Request approval to:\n     
        - Use Camera_2 for area F.\n     
        - Deploy **turtlebot3_2** to area F.\n     
        - Deploy **turtlebot3_1** sequentially to areas G, H, and L."
    
    The actual proposed deployments are 
        - turtlebot3_2 : F     
        - turtlebot3_1 : G
    
    
    2. Do not confuse:
       - Areas being investigated.
       - Possible intruder movements.
       - Robot movements mentioned hypothetically.
       - Actual proposed robot deployments.
    
    3. Only use the robots and areas allowed by the function schema.
    
    4. If multiple deployments are proposed, include all of them.
    
    5. If no deployment is proposed, return an empty list.
    
    6. Do not invent information.
    
    7. You MUST call report_deployments exactly once.
    
    Return the extracted information through the function call.
    """

    tools = [
        {
            "type": "function",
            "name": "report_deployments",
            "description": (
                "Reports the robot deployments explicitly "
                "proposed in the assistant message."
            ),
            "strict": True,
            "parameters": {
                "type": "object",
                "properties": {
                    "deployments": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "robot": {
                                    "type": "string",
                                    "enum": ALLOWED_ROBOTS
                                },
                                "area": {
                                    "type": "string",
                                    "enum": allowed_areas
                                }
                            },
                            "required": [
                                "robot",
                                "area"
                            ],
                            "additionalProperties": False
                        }
                    }
                },
                "required": ["deployments"],
                "additionalProperties": False
            }
        }
    ]

    print(
        f"\n[LLM CALL] Extracting deployments "
        f"for {world_name}",
        flush=True
    )

    response = client.responses.create(
        model=MODEL,
        instructions=system_prompt,
        input=assistant_message,
        tools=tools,
        tool_choice={
            "type": "function",
            "name": "report_deployments"
        },
        parallel_tool_calls=False
    )

    tool_calls = [
        item for item in response.output
        if item.type == "function_call"
        and item.name == "report_deployments"
    ]

    if len(tool_calls) != 1:
        raise ValueError(
            f"Expected exactly one report_deployments call, "
            f"received {len(tool_calls)}."
        )

    arguments = json.loads(tool_calls[0].arguments)

    return arguments["deployments"]



# ============================================================
# DETERMINISTIC DEPLOYMENT STORAGE
# ============================================================

def save_deployment_results(
    run_results: dict,
    turn_number: int,
    deployments: list
):

    turn_name = f"turn_{turn_number}"

    deployment_dict = {}

    for deployment in deployments:

        robot = deployment["robot"]
        area = deployment["area"]

        if robot not in ALLOWED_ROBOTS:
            raise ValueError(f"Invalid robot: {robot}")

        deployment_dict[robot] = area

    run_results[turn_name] = {
        "deployments": deployment_dict
    }


# ============================================================
# SAVE DISTANCES BY WORLD
# ============================================================

def merge_distances(
    distance_data: dict,
    world_name: str,
    new_distances: dict
):
    """
    Adds only new areas and missing robot distances.

    Existing distance values are never overwritten.
    """

    distance_data.setdefault(world_name, {})

    for area, robot_distances in new_distances.items():

        if area not in WORLD_AREAS[world_name]:
            continue

        distance_data[world_name].setdefault(area, {})

        for robot, distance in robot_distances.items():

            if robot not in ALLOWED_ROBOTS:
                continue

            if robot not in distance_data[world_name][area]:

                distance_data[world_name][area][robot] = distance


# ============================================================
# SAVE JSON
# ============================================================

def save_json(data: dict, output_file: Path):

    output_file.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    temporary_file = output_file.with_name(
        output_file.name + ".tmp"
    )

    with temporary_file.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            data,
            file,
            indent=4,
            ensure_ascii=False
        )

    temporary_file.replace(output_file)


# ============================================================
# SELECTION FILTER
# ============================================================

def is_selected(value: str, selection) -> bool:

    if selection is None:
        return True

    return value in selection


# ============================================================
# PROCESS ONE RUN
# ============================================================

def process_run(
    run_data: dict,
    world_name: str,
    experiment_path: str,
    distance_data: dict
) -> dict:

    conversation = run_data.get("conversation", [])

    # --------------------------------------------------------
    # 1. Extract historical distances.
    # --------------------------------------------------------

    computed_distance = extract_computed_distances(
        conversation
    )

    merge_distances(
        distance_data,
        world_name,
        computed_distance
    )

    # --------------------------------------------------------
    # 2. Extract assistant messages.
    # --------------------------------------------------------

    assistant_messages = extract_assistant_messages(
        conversation
    )

    print(f"\nProcessing run: {experiment_path}")
    print(f"Assistant messages: {len(assistant_messages)}")

    run_results = {}

    # --------------------------------------------------------
    # 3. Process EVERY assistant message.
    # --------------------------------------------------------

    for turn_number, assistant_message in enumerate(
        assistant_messages,
        start=1
    ):

        turn_name = f"turn_{turn_number}"

        print(f"\nProcessing {turn_name}")

        # Initialize turn.
        run_results[turn_name] = {
            "deployments": {}
        }

        try:

            deployments = extract_deployments(
                assistant_message,
                world_name
            )

            save_deployment_results(
                run_results,
                turn_number,
                deployments
            )

        except Exception as error:

            print(
                f"[ERROR] {turn_name}: {error}"
            )

            run_results[turn_name] = {
                "deployments": {}
            }

            run_results.setdefault(
                "_manual_check",
                {
                    "reason": "LLM extraction error",
                    "failed_turns": []
                }
            )

            run_results["_manual_check"][
                "failed_turns"
            ].append({
                "turn": turn_name,
                "error": str(error)
            })

    return run_results


# ============================================================
# SAVE ONE RUN IN THE OUTPUT STRUCTURE
# ============================================================

def save_run_result(
    processed_data: dict,
    rag_name: str,
    world_name: str,
    model_name: str,
    trial_name: str,
    run_name: str,
    run_results: dict
):

    processed_data.setdefault(rag_name, {})
    processed_data[rag_name].setdefault(world_name, {})
    processed_data[rag_name][world_name].setdefault(model_name, {})
    processed_data[rag_name][world_name][model_name].setdefault(trial_name, {})

    processed_data[
        rag_name
    ][
        world_name
    ][
        model_name
    ][
        trial_name
    ][
        run_name
    ] = run_results



# ============================================================
# PROCESS ALL EXPERIMENTS
# ============================================================

def process_experiments():

    if not INPUT_FILE.exists():

        raise FileNotFoundError(
            f"Input file not found: {INPUT_FILE}"
        )

    # --------------------------------------------------------
    # 1. Read original JSON.
    # --------------------------------------------------------

    with INPUT_FILE.open(
        "r",
        encoding="utf-8"
    ) as file:

        original_data = json.load(file)

    # --------------------------------------------------------
    # 2. Load existing deployment JSON.
    # --------------------------------------------------------

    if DEPLOYMENTS_FILE.exists():

        print("[INFO] Existing deployment JSON found.")

        with DEPLOYMENTS_FILE.open(
            "r",
            encoding="utf-8"
        ) as file:

            processed_data = json.load(file)

    else:

        print("[INFO] Creating deployment JSON.")

        processed_data = {}

        save_json(
            processed_data,
            DEPLOYMENTS_FILE
        )

    # --------------------------------------------------------
    # 3. Load existing distance JSON.
    # --------------------------------------------------------

    if DISTANCES_FILE.exists():

        print("[INFO] Existing distance JSON found.")

        with DISTANCES_FILE.open(
            "r",
            encoding="utf-8"
        ) as file:

            distance_data = json.load(file)

    else:

        print("[INFO] Creating distance JSON.")

        distance_data = {
            "world_1": {},
            "world_2": {}
        }

        save_json(
            distance_data,
            DISTANCES_FILE
        )

    # Ensure both worlds exist.
    for world_name in WORLD_AREAS:
        distance_data.setdefault(world_name, {})

    # --------------------------------------------------------
    # 4. Iterate through experiment hierarchy.
    # --------------------------------------------------------

    for rag_name, rag_data in original_data.items():

        if not is_selected(
            rag_name,
            SELECTED_MODES
        ):
            continue

        for world_name, world_data in rag_data.items():

            if not is_selected(
                world_name,
                SELECTED_WORLDS
            ):
                continue

            if world_name not in WORLD_AREAS:

                print(
                    f"Skipping unsupported world: {world_name}"
                )

                continue

            for model_name, model_data in world_data.items():

                if not is_selected(
                    model_name,
                    SELECTED_MODELS
                ):
                    continue

                for trial_name, trial_data in model_data.items():

                    if not is_selected(
                        trial_name,
                        SELECTED_TRIALS
                    ):
                        continue

                    for run_name, run_data in trial_data.items():

                        if not isinstance(run_data, dict):
                            continue

                        if "conversation" not in run_data:
                            continue

                        experiment_path = (
                            f"{rag_name}/"
                            f"{world_name}/"
                            f"{model_name}/"
                            f"{trial_name}/"
                            f"{run_name}"
                        )

                        print("\n" + "=" * 60)
                        print(f"RUN: {experiment_path}")
                        print("=" * 60)

                        try:

                            # --------------------------------
                            # Process individual run.
                            # --------------------------------

                            run_results = process_run(
                                run_data,
                                world_name,
                                experiment_path,
                                distance_data
                            )

                            # --------------------------------
                            # Save deployments.
                            # --------------------------------

                            save_run_result(
                                processed_data,
                                rag_name,
                                world_name,
                                model_name,
                                trial_name,
                                run_name,
                                run_results
                            )

                            # --------------------------------
                            # Save BOTH JSON files.
                            # --------------------------------

                            save_json(
                                processed_data,
                                DEPLOYMENTS_FILE
                            )

                            save_json(
                                distance_data,
                                DISTANCES_FILE
                            )

                            print(
                                f"\n[SAVED] {experiment_path}"
                            )

                        except Exception as error:

                            print(
                                f"\n[ERROR] {experiment_path}: {error}"
                            )

                            failed_results = {
                                "_manual_check": {
                                    "reason": "run processing error",
                                    "error": str(error)
                                }
                            }

                            save_run_result(
                                processed_data,
                                rag_name,
                                world_name,
                                model_name,
                                trial_name,
                                run_name,
                                failed_results
                            )

                            save_json(
                                processed_data,
                                DEPLOYMENTS_FILE
                            )

                            save_json(
                                distance_data,
                                DISTANCES_FILE
                            )

    print("\n" + "=" * 60)
    print("EXPERIMENT PROCESSING COMPLETED")
    print(f"Deployments: {DEPLOYMENTS_FILE}")
    print(f"Distances: {DISTANCES_FILE}")
    print("=" * 60)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    process_experiments()
