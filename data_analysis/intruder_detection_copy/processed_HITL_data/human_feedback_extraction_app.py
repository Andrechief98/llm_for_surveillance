import json
import re
from pathlib import Path
from base64 import b64encode

import pandas as pd
import streamlit as st


# ============================================================
# 1. CONFIGURATION
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent

IMAGES_PATH = (
    SCRIPT_DIR / "../../../llm_interface/config/uploads"
).resolve()

WORLD_IMAGES = {
    "world_1": IMAGES_PATH / "building_plan_map1.png",
    "world_2": IMAGES_PATH / "building_plan_map2.png",
}

DATASET_FILE = Path("experiments_HITL_cleared.json")
ANNOTATIONS_FILE = Path("raw_inspection_matrices.json")

RAG_MAP_FILES = {
    "world_1": SCRIPT_DIR / "../../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": SCRIPT_DIR / "../../../llm_interface/config/RAG/RAG_map2.json",
}

st.set_page_config(
    page_title="LLM Conversation Reviewer",
    layout="wide",
    initial_sidebar_state="collapsed",
)


# ============================================================
# 2. STREAMLIT STYLE
# ============================================================

st.markdown(
    """
    <style>
    .stApp {
        font-size: 18px;
    }

    .stApp p,
    .stApp label,
    .stApp .stMarkdown,
    .stApp .stCaption {
        font-size: 18px;
    }

    .stApp h1 {
        font-size: 2.2rem;
    }

    .stApp h2 {
        font-size: 1.8rem;
    }

    .stApp h3 {
        font-size: 1.5rem;
    }

    .stApp input,
    .stApp textarea,
    .stApp [data-baseweb="select"] {
        font-size: 16px;
    }

    .stApp button {
        font-size: 16px;
    }

    .stApp code,
    .stApp pre {
        font-size: 15px;
    }

    .stApp [data-testid="stChatMessage"] p {
        font-size: 18px;
        line-height: 1.6;
    }

    .stApp [data-testid="stChatMessage"] li {
        font-size: 18px;
        line-height: 1.6;
    }

    [data-testid="stDataEditor"] {
        font-size: 14px;
    }
    </style>
    """,
    unsafe_allow_html=True,
)


# ============================================================
# 3. WORLD CONFIGURATION
# ============================================================

# world_1:
# A, B, C, D, E, F, G, H, I, L
#
# world_2:
# A-I, L-W, Z
#
# J, K, X and Y are excluded.

WORLD_AREAS = {
    "world_1": [
        "A", "B", "C", "D", "E",
        "F", "G", "H", "I", "L",
    ],

    "world_2": [
        "A", "B", "C", "D", "E",
        "F", "G", "H", "I",
        "L", "M", "N", "O", "P",
        "Q", "R", "S", "T", "U",
        "V", "W", "Z",
    ],
}


WORLD_DEVICES = {
    "world_1": [
        "robot_1",
        "robot_2",
        "Camera_1",
        "Camera_2",
    ],

    "world_2": [
        "robot_1",
        "robot_2",
        *[
            f"Camera_{i}"
            for i in range(1, 15)
        ],
    ],
}


def load_world_cameras(world_id):
    """
    Load available cameras for each area from the RAG map.

    Returns:
        {
            "A": [],
            "B": ["Camera_1"],
            "F": ["Camera_2"],
            ...
        }
    """

    rag_file = RAG_MAP_FILES.get(world_id)

    if rag_file is None or not rag_file.is_file():

        st.error(
            f"RAG map not found for {world_id}: {rag_file}"
        )

        return {}

    try:

        with rag_file.open("r", encoding="utf-8") as f:
            rag_data = json.load(f)

    except (json.JSONDecodeError, OSError) as exc:

        st.error(
            f"Cannot load RAG map {rag_file}: {exc}"
        )

        return {}

    areas_data = rag_data.get("areas", {})

    if not isinstance(areas_data, dict):

        st.error(
            f"Invalid 'areas' structure in {rag_file}."
        )

        return {}

    area_cameras = {}

    for area in WORLD_AREAS[world_id]:

        area_data = areas_data.get(area, {})

        if not isinstance(area_data, dict):
            area_data = {}

        sensors = area_data.get("sensors", [])

        if not isinstance(sensors, list):
            sensors = []

        area_cameras[area] = [
            sensor
            for sensor in sensors
            if (
                isinstance(sensor, str)
                and sensor.startswith("Camera_")
            )
        ]

    return area_cameras

# ============================================================
# 4. JSON UTILITIES
# ============================================================

def read_json(path, default):
    """
    Read a JSON file.

    If the file does not exist or cannot be read,
    return the provided default value.
    """

    path = Path(path)

    if not path.exists():
        return default

    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    except (json.JSONDecodeError, OSError) as exc:

        st.warning(
            f"Cannot read {path}: {exc}"
        )

        return default


def write_json(path, data):
    """
    Save a JSON file with UTF-8 encoding
    and readable indentation.
    """

    path = Path(path)

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with path.open("w", encoding="utf-8") as f:

        json.dump(
            data,
            f,
            ensure_ascii=False,
            indent=4,
        )


def pretty_content(value):
    """
    Format valid JSON strings.
    Display other contents as plain text.
    """

    if not isinstance(value, str):

        return json.dumps(
            value,
            ensure_ascii=False,
            indent=2,
        )

    try:

        parsed = json.loads(value)

        return json.dumps(
            parsed,
            ensure_ascii=False,
            indent=2,
        )

    except (json.JSONDecodeError, TypeError):

        return value


# ============================================================
# 5. ANNOTATION MATRIX UTILITIES
# ============================================================

def get_empty_matrix(world_id):

    return {
        role: {
            area: ""
            for area in WORLD_AREAS[world_id]
        }
        for role in ("LLM", "human")
    }

def normalize_matrix(matrix, world_id):
    """
    Normalize an existing matrix.

    Guarantees:
    - LLM and human rows exist.
    - All areas of the selected world exist.
    - All values are strings.
    """

    default_matrix = get_empty_matrix(world_id)

    if not isinstance(matrix, dict):
        return default_matrix

    normalized = {}

    for role in ("LLM", "human"):

        role_data = matrix.get(role, {})

        if not isinstance(role_data, dict):
            role_data = {}

        normalized[role] = {}

        for area in WORLD_AREAS[world_id]:

            value = role_data.get(area, "")

            if value is None:
                value = ""

            normalized[role][area] = str(value)

    return normalized



def copy_llm_to_human(matrix, world_id):
    """
    Copy all LLM area selections to the Human plan.

    The copied values remain independently editable.
    The original LLM plan is never modified.
    """

    matrix = normalize_matrix(
        matrix,
        world_id,
    )

    for area in WORLD_AREAS[world_id]:

        matrix["human"][area] = (
            matrix["LLM"][area]
        )

    return matrix


def get_turn_dataframe(world_id, matrix):
    """
    Convert a JSON matrix into a pandas DataFrame
    suitable for Streamlit's data_editor.
    """

    areas = WORLD_AREAS[world_id]

    matrix = normalize_matrix(
        matrix,
        world_id,
    )

    rows = [
        {
            "Role": "LLM",
            **matrix["LLM"],
        },

        {
            "Role": "Human",
            **matrix["human"],
        },
    ]

    return pd.DataFrame(
        rows,
        columns=["Role"] + areas,
    )


def dataframe_to_matrix(dataframe, world_id):
    """
    Convert the edited DataFrame into JSON format.
    """

    areas = WORLD_AREAS[world_id]

    matrix = get_empty_matrix(world_id)

    for _, row in dataframe.iterrows():

        role = str(row["Role"])

        if role == "LLM":
            output_role = "LLM"

        elif role == "Human":
            output_role = "human"

        else:
            continue

        for area in areas:

            value = row[area]

            if pd.isna(value):
                value = ""

            matrix[output_role][area] = str(value)

    return matrix


# ============================================================
# 6. ANNOTATION DATA UTILITIES
# ============================================================

def get_annotation_path(
    annotations,
    condition,
    mode,
    world_id,
    model_id,
    trial_id,
    run_id,
):
    """
    Return the reference to a specific run annotation.

    Creates missing dictionary levels when necessary.
    """

    return (
        annotations
        .setdefault(condition, {})
        .setdefault(mode, {})
        .setdefault(world_id, {})
        .setdefault(model_id, {})
        .setdefault(trial_id, {})
        .setdefault(run_id, {})
    )


def get_turns_from_annotation(annotation):

    matrices = annotation.get("matrix", {})

    if not isinstance(matrices, dict):
        matrices = {}

    turns = {}

    for turn_id, matrix in matrices.items():

        turns[str(turn_id)] = {
            "matrix": matrix,
        }

    return turns

def get_next_turn_number(turns):
    """
    Calculate the next available turn number.
    """

    numbers = []

    for turn_id in turns.keys():

        try:
            numbers.append(int(turn_id))

        except (TypeError, ValueError):
            continue

    return max(numbers, default=0) + 1


def delete_turn(turns, turn_id):
    """
    Delete a turn and renumber the remaining turns.
    """

    remaining_turns = {
        key: value
        for key, value in turns.items()
        if str(key) != str(turn_id)
    }

    renumbered_turns = {}

    for new_number, (_, turn_data) in enumerate(
        sorted(
            remaining_turns.items(),
            key=lambda item: int(item[0])
        ),
        start=1,
    ):

        renumbered_turns[str(new_number)] = turn_data

    return renumbered_turns

def get_annotation_widget_key(
    prefix,
    condition,
    mode,
    world_id,
    model_id,
    trial_id,
    run_id,
    turn_id="",
):
    """
    Generate unique Streamlit widget keys.

    This prevents state conflicts between different runs.
    """

    return (
        f"{prefix}_{condition}_{mode}_{world_id}_"
        f"{model_id}_{trial_id}_{run_id}_{turn_id}"
    )


# ============================================================
# 7. CONVERSATION UTILITIES
# ============================================================

def filter_timing_fields(event):
    """
    Remove timing fields before displaying an event.
    """

    if not isinstance(event, dict):
        return event

    return {
        key: value
        for key, value in event.items()
        if key not in ("resp_time", "gen_time")
    }


# ============================================================
# 8. CAMERA IMAGES FROM ROS OUTPUT
# ============================================================

def render_camera_images_from_ros_output(content):
    """
    Extract image_path fields from ROS output
    and display the corresponding camera frames.
    """

    if not isinstance(content, str):
        return

    try:
        data = json.loads(content)

    except (json.JSONDecodeError, TypeError):
        return

    if not isinstance(data, dict):
        return

    for camera_name, camera_data in data.items():

        if not isinstance(camera_data, dict):
            continue

        image_path = camera_data.get("image_path")

        if (
            not isinstance(image_path, str)
            or not image_path.strip()
        ):
            continue

        relative_path = Path(image_path)

        resolved_path = (
            SCRIPT_DIR / relative_path
        ).resolve()

        st.markdown(
            f"**📷 Camera feed — {camera_name}**"
        )

        if resolved_path.is_file():

            st.image(
                str(resolved_path),
                caption=(
                    f"{camera_name} · "
                    f"{relative_path.as_posix()}"
                ),
                width=400,
            )

        else:

            st.warning(
                f"Camera feed not found: "
                f"{relative_path.as_posix()}"
            )


# ============================================================
# 9. FUNCTION CALL RENDERING
# ============================================================
# def render_area_editor(
#     world_id,
#     matrix,
#     turn_id,
#     role,
#     prefix,
#     area_cameras,
# ):

#     areas = WORLD_AREAS[world_id]

#     st.markdown(f"**{role}**")

#     columns = st.columns(
#         len(areas),
#         gap="small"
#     )

#     result = {}

#     for index, area in enumerate(areas):

#         with columns[index]:

#             st.markdown(f"**{area}**")

#             # --------------------------------------------
#             # Available devices for this area
#             # --------------------------------------------

#             available_devices = (
#                 ["robot_1", "robot_2"]
#                 + area_cameras.get(area, [])
#             )

#             devices = [""] + available_devices

#             # --------------------------------------------
#             # Previous annotation
#             # --------------------------------------------

#             old_value = matrix.get(
#                 role, {}
#             ).get(
#                 area, ""
#             )

#             # --------------------------------------------
#             # Validate previous annotation
#             # --------------------------------------------

#             if old_value not in devices:

#                 old_value = ""

#             # --------------------------------------------
#             # Widget
#             # --------------------------------------------

#             device_key = (
#                 f"{prefix}_{turn_id}_{role}_{area}"
#             )

#             # Reset invalid widget state if necessary
#             if (
#                 device_key in st.session_state
#                 and st.session_state[device_key] not in devices
#             ):

#                 st.session_state[device_key] = ""

#             device = st.selectbox(
#                 "Device",
#                 options=devices,
#                 index=devices.index(old_value),
#                 key=device_key,
#                 label_visibility="collapsed",
#             )

#             result[area] = device

#     return result


def render_area_editor(
    world_id,
    matrix,
    turn_id,
    role,
    prefix,
    area_cameras,
):

    areas = WORLD_AREAS[world_id]

    st.markdown(f"**{role}**")

    # --------------------------------------------
    # Divide areas into rows
    # --------------------------------------------

    if world_id == "world_2":
        areas_per_row = 12
    else:
        areas_per_row = len(areas)

    result = {}

    # --------------------------------------------
    # Render areas row by row
    # --------------------------------------------

    for start in range(0, len(areas), areas_per_row):

        row_areas = areas[start:start + areas_per_row]

        columns = st.columns(
            len(row_areas),
            gap="small"
        )

        for index, area in enumerate(row_areas):

            with columns[index]:

                st.markdown(f"**{area}**")

                # Available devices for this area
                available_devices = (
                    ["robot_1", "robot_2"]
                    + area_cameras.get(area, [])
                )

                devices = [""] + available_devices

                # Previous annotation
                old_value = matrix.get(
                    role, {}
                ).get(
                    area, ""
                )

                # Validate previous annotation
                if old_value not in devices:
                    old_value = ""

                # Widget key
                device_key = (
                    f"{prefix}_{turn_id}_{role}_{area}"
                )

                # Reset invalid widget state
                if (
                    device_key in st.session_state
                    and st.session_state[device_key] not in devices
                ):
                    st.session_state[device_key] = ""

                # Device selector
                device = st.selectbox(
                    "Device",
                    options=devices,
                    index=devices.index(old_value),
                    key=device_key,
                    label_visibility="collapsed",
                )

                result[area] = device

    return result


def render_function_call(content, index):
    """
    Display a function call and its arguments.
    """

    with st.container(border=True):

        st.markdown(
            f"🔧 **FUNCTION CALL** · Event {index + 1}"
        )

        function_name = "Unknown function"
        arguments = {}

        if isinstance(content, str):

            for line in content.splitlines():

                line = line.strip()

                if line.startswith("name - "):

                    function_name = (
                        line[len("name - "):].strip()
                    )

                elif line.startswith("arguments - "):

                    arguments_str = (
                        line[len("arguments - "):].strip()
                    )

                    try:

                        arguments = json.loads(
                            arguments_str
                        )

                    except json.JSONDecodeError:

                        arguments = arguments_str

        else:

            arguments = content

        st.markdown(
            f"**Function:** `{function_name}`"
        )

        st.markdown(
            "**Parameters:**"
        )

        if isinstance(arguments, dict):

            if arguments:

                for key, value in arguments.items():

                    if isinstance(value, (dict, list)):

                        value = json.dumps(
                            value,
                            ensure_ascii=False,
                        )

                    st.markdown(
                        f"- `{key}`: `{value}`"
                    )

            else:

                st.caption("No parameters.")

        else:

            st.code(
                pretty_content(arguments),
                language="json",
            )


# ============================================================
# 10. ROS OUTPUT RENDERING
# ============================================================

def render_ros_output(content, index):
    """
    Display ROS output and associated camera images.
    """

    with st.container(border=True):

        st.markdown(
            f"⚙️ **ROS OUTPUT** · Event {index + 1}"
        )

        with st.expander("Show ROS output"):

            st.code(
                pretty_content(content),
                language="json",
            )

        render_camera_images_from_ros_output(
            content
        )


# ============================================================
# 11. TOOL OUTPUT RENDERING
# ============================================================

def render_tool_output(content, index):

    with st.container(border=True):

        st.markdown(
            f"🧰 **TOOL OUTPUT** · Event {index + 1}"
        )

        with st.expander("Show result"):

            st.code(
                pretty_content(content)
            )


# ============================================================
# 12. DATA ORCHESTRATOR RENDERING
# ============================================================

def render_data_orchestrator(content, index):
    """
    Extract and display the current environment state
    from a data_orchestrator message.
    """

    with st.container(border=True):

        st.markdown(
            f"🚨 **DATA ORCHESTRATOR** · Event {index + 1}"
        )

        if not isinstance(content, str):

            st.code(
                pretty_content(content)
            )

            return

        match = re.search(
            r"Current environment state:\s*(\{.*?\})\.\s*Propose",
            content,
            re.DOTALL,
        )

        if not match:

            st.warning(
                "Environment state not found."
            )

            return

        try:

            decoder = json.JSONDecoder()

            state, _ = decoder.raw_decode(
                match.group(1)
            )

            robots = state.get(
                "robots",
                {},
            )

            activated_sensors = state.get(
                "activated_sensors",
                [],
            )

            # --------------------------------------------
            # Active sensors
            # --------------------------------------------

            st.markdown(
                "#### 📡 Active sensors"
            )

            if activated_sensors:

                st.markdown(
                    " · ".join(
                        f"`{sensor}`"
                        for sensor in activated_sensors
                    )
                )

            else:

                st.caption(
                    "No active sensors."
                )

            # --------------------------------------------
            # Robots
            # --------------------------------------------

            st.markdown(
                "#### 🤖 Robots"
            )

            if robots:

                for robot_name, robot_info in robots.items():

                    if not isinstance(robot_info, dict):
                        continue

                    location = robot_info.get(
                        "current_area",
                        "N/D",
                    )

                    status = robot_info.get(
                        "status",
                        "N/D",
                    )

                    st.markdown(
                        f"- **{robot_name}** · "
                        f"📍 Area: `{location}` · "
                        f"Status: {status}"
                    )

            else:

                st.caption(
                    "No robot available."
                )

        except (
            json.JSONDecodeError,
            TypeError,
            AttributeError,
        ):

            st.warning(
                "Cannot interpret environment state."
            )


# ============================================================
# 13. GENERIC EVENT RENDERING
# ============================================================

def render_event(event, index):
    """
    Render a conversation event.

    Layout:
        Assistant -> left
        User      -> right
        Tools     -> bordered containers
    """

    if not isinstance(event, dict):

        with st.container(border=True):

            st.caption(
                f"📄 EVENT {index + 1}"
            )

            st.code(
                pretty_content(event)
            )

        return

    event = filter_timing_fields(event)

    for role, content in event.items():

        role_lower = str(role).lower()

        # ----------------------------------------------------
        # ASSISTANT
        # ----------------------------------------------------

        if role_lower == "assistant":

            left, right = st.columns([1, 1])

            with left:

                with st.chat_message(
                    "assistant",
                    avatar="🤖",
                ):

                    st.markdown(
                        str(content)
                    )

        # ----------------------------------------------------
        # USER
        # ----------------------------------------------------

        elif role_lower == "user":

            left, right = st.columns([1, 1])

            with right:

                with st.chat_message(
                    "user",
                    avatar="👤",
                ):

                    st.markdown(
                        str(content)
                    )

        # ----------------------------------------------------
        # FUNCTION CALL
        # ----------------------------------------------------

        elif role_lower == "function_call":

            render_function_call(
                content,
                index,
            )

        # ----------------------------------------------------
        # ROS OUTPUT
        # ----------------------------------------------------

        elif role_lower == "ros_output":

            render_ros_output(
                content,
                index,
            )

        # ----------------------------------------------------
        # TOOL OUTPUT
        # ----------------------------------------------------

        elif role_lower == "tool":

            render_tool_output(
                content,
                index,
            )

        # ----------------------------------------------------
        # DATA ORCHESTRATOR
        # ----------------------------------------------------

        elif role_lower == "data_orchestrator":

            render_data_orchestrator(
                content,
                index,
            )

        # ----------------------------------------------------
        # OTHER ROLES
        # ----------------------------------------------------

        else:

            with st.container(border=True):

                st.markdown(
                    f"📄 **{str(role).upper()}** · "
                    f"Event {index + 1}"
                )

                with st.expander("Show content"):

                    st.code(
                        pretty_content(content)
                    )


# ============================================================
# 14. WORLD IMAGE RENDERING
# ============================================================

def render_world_image(world_id):
    """
    Display the building plan centered in its column.
    """

    reference_image = WORLD_IMAGES.get(
        str(world_id)
    )

    if (
        reference_image is None
        or not reference_image.is_file()
    ):

        st.warning(
            f"Building plan not found for {world_id}."
        )

        return

    image_bytes = reference_image.read_bytes()

    image_base64 = b64encode(
        image_bytes
    ).decode("utf-8")

    width = (
        1500
        if world_id == "world_2"
        else 700
    )

    st.markdown(
        f"""
        <div style="
            width: 100%;
            display: flex;
            justify-content: center;
            align-items: center;
            overflow: hidden;
        ">
            <img
                src="data:image/png;base64,{image_base64}"
                style="
                    width: {width}px;
                    max-width: 100%;
                    height: auto;
                    object-fit: contain;
                    display: block;
                    margin: 0 auto;
                "
            />
        </div>
        """,
        unsafe_allow_html=True,
    )

    st.caption(
        f"Building plan — {world_id}"
    )


# ============================================================
# 15. CONVERSATION STATISTICS
# ============================================================

def count_message_types(conversation):
    """
    Count events and group them by sender type.
    """

    counts = {}

    for event in conversation:

        if isinstance(event, dict):

            if not event:

                counts["empty_event"] = (
                    counts.get("empty_event", 0) + 1
                )

                continue

            for role in event:

                counts[role] = (
                    counts.get(role, 0) + 1
                )

        else:

            counts["unstructured_event"] = (
                counts.get("unstructured_event", 0) + 1
            )

    return len(conversation), counts



# ============================================================
# 16. APPLICATION HEADER
# ============================================================

st.title("LLM Conversation Reviewer")

st.caption(
    "Human-LLM interaction analysis and plan annotation"
)


# ============================================================
# 17. LOAD ORIGINAL DATASET
# ============================================================

if not DATASET_FILE.is_file():

    st.error(
        f"Dataset not found: {DATASET_FILE.resolve()}"
    )

    st.stop()


try:

    with DATASET_FILE.open(
        "r",
        encoding="utf-8",
    ) as f:

        dataset = json.load(f)

except (json.JSONDecodeError, OSError) as exc:

    st.error(
        f"Cannot load dataset: {exc}"
    )

    st.stop()


if not isinstance(dataset, dict) or not dataset:

    st.error(
        "The dataset must contain a non-empty JSON object."
    )

    st.stop()


# ============================================================
# 18. LOAD ANNOTATIONS
# ============================================================

annotations = read_json(
    ANNOTATIONS_FILE,
    {},
)

if not isinstance(annotations, dict):

    annotations = {}


# ============================================================
# 19. NAVIGATION
# ============================================================

st.subheader("Navigation")


# ------------------------------------------------------------
# CONDITION
# ------------------------------------------------------------

available_conditions = [
    condition
    for condition in ("deterministic", "uncertain")
    if isinstance(
        dataset.get(condition),
        dict,
    )
]


if not available_conditions:

    st.error(
        "No valid deterministic/uncertain condition."
    )

    st.stop()


nav_selectors = st.columns(
    [1.2, 1, 0.8, 1.2, 0.8],
    gap="small",
)


with nav_selectors[0]:

    condition = st.selectbox(
        "Condition",
        available_conditions,
        key="selected_condition",
    )


condition_data = dataset[condition]


# ------------------------------------------------------------
# MODALITY
# ------------------------------------------------------------

available_modes = [
    mode_id
    for mode_id in ("HITL", "AUTO")
    if isinstance(
        condition_data.get(mode_id),
        dict,
    )
]


if not available_modes:

    st.error(
        f"No HITL/AUTO modality in {condition}."
    )

    st.stop()


with nav_selectors[1]:

    mode = st.selectbox(
        "Modality",
        available_modes,
        key=f"selected_mode_{condition}",
    )


mode_data = condition_data[mode]


# ------------------------------------------------------------
# WORLD
# ------------------------------------------------------------

available_worlds = [
    str(world_id)
    for world_id, world_data in mode_data.items()
    if (
        isinstance(world_data, dict)
        and str(world_id) in WORLD_AREAS
    )
]


if not available_worlds:

    st.error(
        f"No supported world available for "
        f"{condition}/{mode}."
    )

    st.stop()


with nav_selectors[2]:

    world_id = st.selectbox(
        "World",
        available_worlds,
        key=f"selected_world_{condition}_{mode}",
    )


world_data = mode_data[world_id]

# ============================================================
# LOAD AVAILABLE CAMERAS FROM RAG MAP
# ============================================================

AREA_CAMERAS = load_world_cameras(world_id)

# ------------------------------------------------------------
# MODEL
# ------------------------------------------------------------

available_models = [
    str(model_id)
    for model_id, model_data in world_data.items()
    if isinstance(model_data, dict)
]


if not available_models:

    st.warning(
        f"No model for {condition}/{mode}/{world_id}."
    )

    st.stop()


with nav_selectors[3]:

    model_id = st.selectbox(
        "Model",
        available_models,
        key=(
            f"selected_model_{condition}_"
            f"{mode}_{world_id}"
        ),
    )


model_data = world_data[model_id]


# ------------------------------------------------------------
# TRIAL
# ------------------------------------------------------------

available_trials = [
    str(trial_id)
    for trial_id, trial_data in model_data.items()
    if isinstance(trial_data, dict)
]


if not available_trials:

    st.warning(
        "No trial available."
    )

    st.stop()


with nav_selectors[4]:

    trial_id = st.selectbox(
        "Trial",
        available_trials,
        key=(
            f"selected_trial_{condition}_"
            f"{mode}_{world_id}_{model_id}"
        ),
    )


trial_data = model_data[trial_id]


# ============================================================
# 20. RUN SELECTION
# ============================================================

valid_runs = {
    str(run_id): run
    for run_id, run in trial_data.items()
    if (
        isinstance(run, dict)
        and isinstance(
            run.get("conversation"),
            list,
        )
    )
}


if not valid_runs:

    st.warning(
        f"Trial {trial_id} does not contain valid runs "
        "with a 'conversation' list."
    )

    st.stop()


world_runs = list(valid_runs.keys())


# ============================================================
# 21. RUN NAVIGATION STATE
# ============================================================

nav_key = (
    f"run_index_{condition}_{mode}_{world_id}_"
    f"{model_id}_{trial_id}"
)


if nav_key not in st.session_state:

    st.session_state[nav_key] = 0


# Ensure the index is within valid limits.

st.session_state[nav_key] = min(
    max(
        0,
        st.session_state[nav_key],
    ),
    len(world_runs) - 1,
)


# ============================================================
# 22. PREVIOUS / NEXT BUTTONS
# ============================================================

nav_left, nav_center, nav_right = st.columns(
    [1, 2, 1]
)


with nav_left:

    if st.button(
        "← Previous",
        disabled=(
            st.session_state[nav_key] == 0
        ),
        use_container_width=True,
        key=(
            f"prev_{condition}_{mode}_"
            f"{world_id}_{model_id}_{trial_id}"
        ),
    ):

        st.session_state[nav_key] -= 1

        st.rerun()


with nav_right:

    if st.button(
        "Next →",
        disabled=(
            st.session_state[nav_key]
            >= len(world_runs) - 1
        ),
        use_container_width=True,
        key=(
            f"next_{condition}_{mode}_"
            f"{world_id}_{model_id}_{trial_id}"
        ),
    ):

        st.session_state[nav_key] += 1

        st.rerun()


# ============================================================
# 23. CURRENT RUN
# ============================================================

run_index = st.session_state[nav_key]

run_id = world_runs[run_index]

run = valid_runs[run_id]


with nav_center:

    st.markdown(
        f"""
        <div style="text-align:center">
            <b>{run_id}</b><br>
            {run_index + 1} of {len(world_runs)}
        </div>
        """,
        unsafe_allow_html=True,
    )


st.caption(
    f"Current selection: "
    f"{condition} / {mode} / {world_id} / "
    f"{model_id} / {trial_id} / {run_id}"
)


# ============================================================
# 24. CURRENT RUN ANNOTATION
# ============================================================

current_annotation = get_annotation_path(
    annotations=annotations,
    condition=condition,
    mode=mode,
    world_id=world_id,
    model_id=model_id,
    trial_id=trial_id,
    run_id=run_id,
)

# ------------------------------------------------------------
# Copy metadata from original dataset
# ------------------------------------------------------------

current_annotation["sensor_sequence"] = run.get(
    "sensor_sequence",
    [],
)

current_annotation["success"] = run.get(
    "success",
    None,
)

current_annotation["intruder_final_area"] = run.get(
    "intruder_final_area",
    None,
)


# ------------------------------------------------------------
# Ensure correct structure
# ------------------------------------------------------------

if not isinstance(
    current_annotation.get("matrix"),
    dict,
):

    current_annotation["matrix"] = {}



# ------------------------------------------------------------
# Recover existing annotations
# ------------------------------------------------------------

saved_turns = get_turns_from_annotation(
    current_annotation
)


# ------------------------------------------------------------
# Number of saved turns
# ------------------------------------------------------------

number_of_saved_turns = len(saved_turns)


with st.container():

    if number_of_saved_turns > 0:

        st.success(
            f"🟢 {number_of_saved_turns} annotated turn(s) saved."
        )

    else:

        st.info(
            "No annotations available for this run."
        )


# ============================================================
# 25. TURN MANAGEMENT INITIALIZATION
# ============================================================

turn_counter_key = get_annotation_widget_key(
    prefix="turn_count",
    condition=condition,
    mode=mode,
    world_id=world_id,
    model_id=model_id,
    trial_id=trial_id,
    run_id=run_id,
)

active_turns_key = get_annotation_widget_key(
    prefix="active_turns",
    condition=condition,
    mode=mode,
    world_id=world_id,
    model_id=model_id,
    trial_id=trial_id,
    run_id=run_id,
)

# Initialize active turns only once.
# Deleted turns will not be recreated.

if active_turns_key not in st.session_state:

    st.session_state[active_turns_key] = list(
        saved_turns.keys()
    )

# Initialize the next available ID.

if turn_counter_key not in st.session_state:

    existing_numbers = [
        int(turn_id)
        for turn_id in saved_turns
        if str(turn_id).isdigit()
    ]

    st.session_state[turn_counter_key] = (
        max(existing_numbers, default=0) + 1
    )

# ============================================================
# 26. MAIN LAYOUT
# ============================================================

left, right = st.columns(
    [1.3, 1.7],
    gap="large",
)


# ============================================================
# 27. LEFT COLUMN: BUILDING PLAN
# ============================================================

with left:

    # st.subheader(
    #     "Reference building plan"
    # )

    # render_world_image(world_id)


    # --------------------------------------------------------
    # Conversation information
    # --------------------------------------------------------

    st.subheader(
        "Conversation"
    )

    st.caption(
        f"Modality: {mode} · "
        f"Trial: {trial_id} · "
        f"World: {world_id} · "
        f"Run: {run_id} · "
        f"Model: {model_id}"
    )


    # --------------------------------------------------------
    # Original sensor sequence
    # --------------------------------------------------------

    original_sensor_sequence = run.get(
        "sensor_sequence",
        [],
    )

    st.markdown(
        "**Original sensor sequence**"
    )

    if original_sensor_sequence:

        st.markdown(
            " · ".join(
                f"`{sensor}`"
                for sensor in original_sensor_sequence
            )
        )

    else:

        st.caption(
            "No sensor sequence available in the original dataset."
        )


    # --------------------------------------------------------
    # Conversation statistics
    # --------------------------------------------------------

    conversation = run.get(
        "conversation",
        [],
    )

    total_messages, message_counts = count_message_types(
        conversation
    )

    with st.expander(
        "Message statistics",
        expanded=False,
    ):

        st.metric(
            "Total events",
            total_messages,
        )

        sorted_counts = sorted(
            message_counts.items(),
            key=lambda item: item[0].lower(),
        )

        if sorted_counts:

            stats_columns = st.columns(
                min(len(sorted_counts), 3)
            )

            for index, (role, count) in enumerate(
                sorted_counts
            ):

                with stats_columns[
                    index % len(stats_columns)
                ]:

                    st.metric(
                        label=role,
                        value=count,
                    )


    # --------------------------------------------------------
    # Conversation history
    # --------------------------------------------------------

    st.markdown(
        "### Conversation history"
    )

    with st.container(
        height=1250,
        border=True,
    ):

        for index, event in enumerate(conversation):

            render_event(
                event,
                index,
            )


# ============================================================
# 28. RIGHT COLUMN: PLAN ANNOTATION
# ============================================================

with right:

    st.subheader(
        "Plan annotation"
    )

    st.caption(
        "Annotate the plan proposed by the LLM "
        "and the modifications introduced by the human."
    )


    # ========================================================
    # 29. ADD NEW TURN
    # ========================================================

    st.divider()


    # ========================================================
    # 30. AVAILABLE AREAS AND DEVICES
    # ========================================================

    # with st.expander(
    #     "World configuration",
    #     expanded=False,
    # ):

    #     st.markdown(
    #         f"**World:** `{world_id}`"
    #     )

    #     st.markdown(
    #         "**Available areas:**"
    #     )

    #     st.write(
    #         WORLD_AREAS[world_id]
    #     )

    #     st.markdown(
    #         "**Available devices:**"
    #     )

    #     st.write(
    #         WORLD_DEVICES[world_id]
    #     )


    # ========================================================
    # 31. RENDER ALL TURNS
    # ========================================================

    edited_turns = {}

    # Get ONLY the active turns.
    # Do not recreate IDs using range().

    turn_ids = st.session_state[active_turns_key]

    # Sort turns numerically without renumbering them.

    turn_ids = sorted(
        turn_ids,
        key=int,
    )

    for turn_id in turn_ids:

        turn_number = int(turn_id)

        # ----------------------------------------------------
        # Recover existing annotation
        # ----------------------------------------------------

        old_turn = saved_turns.get(
            turn_id,
            {},
        )

        if not isinstance(old_turn, dict):
            old_turn = {}

        old_matrix = normalize_matrix(
            old_turn.get(
                "matrix",
                get_empty_matrix(world_id),
            ),
            world_id,
        )

        # ----------------------------------------------------
        # Turn container
        # ----------------------------------------------------

        with st.container(border=True):

            title_col, delete_col = st.columns(
                [5, 1]
            )

            with title_col:

                st.markdown(
                    f"### Turn {turn_number}"
                )

            with delete_col:

                if st.button(
                    "🗑️",
                    key=f"delete_turn_{turn_counter_key}_{turn_id}",
                    help=f"Delete only turn {turn_number}",
                    use_container_width=True,
                ):

                    # ----------------------------------------
                    # 1. Remove ONLY the selected turn
                    # ----------------------------------------

                    st.session_state[active_turns_key] = [
                        t
                        for t in st.session_state[active_turns_key]
                        if str(t) != str(turn_id)
                    ]

                    # ----------------------------------------
                    # 2. Remove ONLY its saved annotation
                    # ----------------------------------------

                    current_annotation["matrix"].pop(
                        str(turn_id),
                        None,
                    )

                    # ----------------------------------------
                    # 3. Remove its widget states
                    # ----------------------------------------

                    prefixes = (
                        f"{condition}_{mode}_{world_id}_"
                        f"{model_id}_{trial_id}_{run_id}"
                    )

                    for role in ("LLM", "human"):

                        for area in WORLD_AREAS[world_id]:

                            widget_key = (
                                f"{prefixes}_{turn_id}_{role}_{area}"
                            )

                            st.session_state.pop(
                                widget_key,
                                None,
                            )

                    # ----------------------------------------
                    # 4. Save immediately
                    # ----------------------------------------

                    write_json(
                        ANNOTATIONS_FILE,
                        annotations,
                    )

                    # ----------------------------------------
                    # 5. Refresh the interface
                    # ----------------------------------------

                    st.rerun()

            # =================================================
            # MATRIX EDITOR
            # =================================================

            st.markdown("### LLM")

            llm_matrix = render_area_editor(
                world_id,
                old_matrix,
                turn_id,
                "LLM",
                (
                    f"{condition}_{mode}_{world_id}_"
                    f"{model_id}_{trial_id}_{run_id}"
                ),
                AREA_CAMERAS,
            )

            st.divider()

            # =================================================
            # COPY LLM PLAN TO HUMAN
            # =================================================

            copy_col1, copy_col2, copy_col3 = st.columns(
                [1, 2, 1]
            )

            with copy_col2:

                if st.button(
                    "↓ Copy LLM plan to Human",
                    key=f"copy_llm_human_{turn_counter_key}_{turn_id}",
                    use_container_width=True,
                    help="Copy all LLM area selections to the Human plan.",
                ):

                    # Copy current LLM selections.
                    # The Human plan remains independently editable.

                    for area in WORLD_AREAS[world_id]:

                        llm_key = (
                            f"{condition}_{mode}_{world_id}_"
                            f"{model_id}_{trial_id}_{run_id}_"
                            f"{turn_id}_LLM_{area}"
                        )

                        human_key = (
                            f"{condition}_{mode}_{world_id}_"
                            f"{model_id}_{trial_id}_{run_id}_"
                            f"{turn_id}_human_{area}"
                        )

                        st.session_state[human_key] = (
                            st.session_state.get(
                                llm_key,
                                "",
                            )
                        )

                    st.rerun()


            st.markdown("### Human")

            human_matrix = render_area_editor(
                world_id,
                old_matrix,
                turn_id,
                "human",
                (
                    f"{condition}_{mode}_{world_id}_"
                    f"{model_id}_{trial_id}_{run_id}"
                ),
                AREA_CAMERAS,
            )
            matrix = {
                "LLM": llm_matrix,
                "human": human_matrix,
            }

            edited_turns[turn_id] = {
                "matrix": matrix,
            }

            

    # ========================================================
    # ADD NEW TURN
    # ========================================================

    st.divider()

    if st.button(
        "＋ Add new turn",
        type="primary",
        use_container_width=True,
        key=f"add_{turn_counter_key}",
    ):

        # Get a new unique ID
        new_turn_id = str(
            st.session_state[turn_counter_key]
        )

        # Add only the new turn
        st.session_state[active_turns_key].append(
            new_turn_id
        )

        # Increment the ID for the next turn
        st.session_state[turn_counter_key] += 1

        st.rerun()

    # ========================================================
    # 34. ANNOTATION SUMMARY
    # ========================================================

    # st.divider()

    # st.subheader(
    #     "Annotation summary"
    # )

    # st.metric(
    #     label="Total turns",
    #     value=number_of_turns,
    # )


    # Count non-empty cells in the current editor state.

    total_llm_actions = 0
    total_human_actions = 0

    for turn_data in edited_turns.values():

        matrix = turn_data["matrix"]

        total_llm_actions += sum(
            1
            for value in matrix["LLM"].values()
            if value
        )

        total_human_actions += sum(
            1
            for value in matrix["human"].values()
            if value
        )

    summary_col1, summary_col2 = st.columns(2)

    with summary_col1:

        st.metric(
            "LLM actions",
            total_llm_actions,
        )

    with summary_col2:

        st.metric(
            "Human actions",
            total_human_actions,
        )


    # ========================================================
    # 35. SAVE ANNOTATIONS
    # ========================================================

    st.divider()

    st.subheader(
        "Save annotations"
    )

    st.caption(
        "Save all turns associated with the current run."
    )


    if st.button(
        "💾 Save annotations",
        type="primary",
        use_container_width=True,
        key=(
            f"save_annotations_{condition}_{mode}_"
            f"{world_id}_{model_id}_{trial_id}_{run_id}"
        ),
    ):

        try:

            # -----------------------------------------------
            # Save matrices
            # -----------------------------------------------

            current_annotation["matrix"] = {
                turn_id: turn_data["matrix"]
                for turn_id, turn_data in edited_turns.items()
            }




            # -----------------------------------------------
            # Save JSON file
            # -----------------------------------------------

            write_json(
                ANNOTATIONS_FILE,
                annotations,
            )


            st.success(
                "Annotations successfully saved!"
            )

            st.caption(
                f"Saved file: {ANNOTATIONS_FILE.resolve()}"
            )

            st.rerun()


        except (OSError, TypeError, ValueError) as exc:

            st.error(
                f"Error while saving annotations: {exc}"
            )


# ============================================================
# END OF APPLICATION
# ============================================================