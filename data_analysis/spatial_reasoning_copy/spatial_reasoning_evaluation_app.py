import json
from pathlib import Path

import streamlit as st
from streamlit_ace import st_ace
import os
import re
import ast


script_dir = os.path.dirname(__file__) #/home/[...]/llm_for_surveillance/llm_data_analysis/spatial_reasoning
images_path = os.path.join(script_dir, "../../llm_interface/config/uploads")

WORLD_IMAGES = {
    "world_1": os.path.join(images_path, "building_plan_map1.png"),
    "world_2": os.path.join(images_path, "building_plan_map2.png"),
}


st.set_page_config(
    page_title="LLM Conversation Reviewer",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# ---------- Personalizzazione font ----------

st.markdown(
    """
    <style>
    /* General text of the application */
    .stApp {
        font-size: 18px;
    }

    /* Text of paragraphs, labels and descriptions */
    .stApp p,
    .stApp label,
    .stApp .stMarkdown,
    .stApp .stCaption {
        font-size: 18px;
    }

    /* Titles and sections */
    .stApp h1 {
        font-size: 2.2rem;
    }

    .stApp h2 {
        font-size: 1.8rem;
    }

    .stApp h3 {
        font-size: 1.5rem;
    }

    /* Text and menù */
    .stApp input,
    .stApp textarea,
    .stApp [data-baseweb="select"] {
        font-size: 17px;
    }

    /* Buttons */
    .stApp button {
        font-size: 17px;
    }

    /* Code blocks */
    .stApp code,
    .stApp pre {
        font-size: 15px;
    }

    /* Assistant and user messages */
    .stApp [data-testid="stChatMessage"] p {
        font-size: 19px;
        line-height: 1.6;
    }

    /* Markdown readability of the responses */
    .stApp [data-testid="stChatMessage"] li {
        font-size: 19px;
        line-height: 1.6;
    }

    /* Markdown title within messages */
    .stApp [data-testid="stChatMessage"] h1 {
        font-size: 1.7rem;
    }

    .stApp [data-testid="stChatMessage"] h2 {
        font-size: 1.5rem;
    }

    .stApp [data-testid="stChatMessage"] h3 {
        font-size: 1.3rem;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

EVALUATIONS_FILE = Path("evaluations.json")
DATASET_FILE = Path("experiments.json")
CATEGORIES_FILE = Path("error_categories.json")

DEFAULT_CATEGORIES = {
  "Reasoning_errors": [
    "Spatial reasoning errors"
  ],
  "Hallucinations": [
    "get_areas_info with useless areas",
    "get_sensors_info with useless sensors"
  ],
  "Wrong function calls": [
    "Unapproved action execution",
    "Wrong parameters generated"
  ]
}


# ---------- Utility ----------

def read_json(path, default):
    if not path.exists():
        return default

    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return default


def write_json(path, data):
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def pretty_content(value):
    """Formatting valid JSON; show other contents as text."""
    if not isinstance(value, str):
        return json.dumps(value, ensure_ascii=False, indent=2)

    try:
        parsed = json.loads(value)
        return json.dumps(parsed, ensure_ascii=False, indent=2)
    except (json.JSONDecodeError, TypeError):
        return value


def render_event(event, index):
    """Assistant on the left, user on the right."""

    if not isinstance(event, dict):
        with st.container(border=True):
            st.caption(f"📄 EVENT {index + 1}")
            st.code(pretty_content(event))
        return

    for role, content in event.items():
        role_lower = role.lower()

        if role_lower == "assistant":
            # Assistant message
            left, right = st.columns([1, 1])

            with left:
                with st.chat_message("assistant", avatar="🤖"):
                    st.markdown(str(content))

        elif role_lower == "user":
            # User message
            left, right = st.columns([1, 1])

            with right:
                with st.chat_message("user", avatar="👤"):
                    st.markdown(str(content))

        elif role_lower == "function_call":
            with st.container(border=True):
                st.markdown(f"🔧 **FUNCTION CALL** · Event {index + 1}")

                function_name = "Unknown function"
                arguments = {}

                for line in content.splitlines():
                    line = line.strip()

                    if line.startswith("name - "):
                        function_name = line[len("name - "):].strip()

                    elif line.startswith("arguments - "):
                        arguments_str = line[len("arguments - "):].strip()

                        try:
                            arguments = json.loads(arguments_str)
                        except json.JSONDecodeError:
                            arguments = arguments_str

                st.markdown(f"**Function:** `{function_name}`")
                st.markdown("**Parameters:**")

                if isinstance(arguments, dict):
                    if arguments:
                        for key, value in arguments.items():
                            if isinstance(value, (dict, list)):
                                value = json.dumps(value, ensure_ascii=False)
                            st.markdown(f"- `{key}`: `{value}`")
                    else:
                        st.caption("No parameters")
                else:
                    st.code(str(arguments), language="json")

        elif role_lower == "ros_output":
            with st.container(border=True):
                st.markdown(
                    f"⚙️ **ROS OUTPUT** · Event {index + 1}"
                )
                with st.expander("Show ROS output"):
                    st.code(pretty_content(content))

        elif role_lower == "tool":
            with st.container(border=True):
                st.markdown(
                    f"🧰 **TOOL OUTPUT** · Event {index + 1}"
                )
                with st.expander("Show result"):
                    st.code(pretty_content(content))


        elif role_lower == "data_orchestrator":
            with st.container(border=True):
                st.markdown(f"🚨 **DATA ORCHESTRATOR** · Event {index + 1}")

                match = re.search(
                    r"Current environment state:\s*(\{.*?\})\.\s*Propose",
                    content,
                    re.DOTALL
                )

                if match:
                    try:
                        decoder = json.JSONDecoder()
                        state, _ = decoder.raw_decode(match.group(1))

                        robots = state.get("robots", {})
                        activated_sensors = state.get("activated_sensors", [])

                        st.markdown("#### 📡 Active sensors")

                        if activated_sensors:
                            st.markdown(
                                " · ".join(
                                    f"`{sensor}`" for sensor in activated_sensors
                                )
                            )
                        else:
                            st.caption("No active sensors.")

                        st.markdown("#### 🤖 Robot")

                        if robots:
                            for robot_name, robot_info in robots.items():
                                location = robot_info.get("current_area", "N/D")
                                status = robot_info.get("status", "N/D")

                                st.markdown(
                                    f"- **{robot_name}** · "
                                    f"📍 Area: `{location}` · "
                                    f"Status: {status}"
                                )
                        else:
                            st.caption("No robot present.")


                    except (json.JSONDecodeError, TypeError):
                        st.warning("Impossible to interpret environmental state.")
                else:
                    st.warning("Environmental state not found in the conversation.")

        else:
            with st.container(border=True):
                st.markdown(
                    f"📄 **{role.upper()}** · Event {index + 1}"
                )
                with st.expander("Show content"):
                    st.code(pretty_content(content))

def count_message_types(conversation):
    """Count the total messages and gather for type of sender."""
    counts = {}

    for event in conversation:
        if isinstance(event, dict):
            if not event:
                counts["empty_event"] = counts.get("empty_event", 0) + 1
                continue

            # Each key is a sender.
            for role in event:
                counts[role] = counts.get(role, 0) + 1
        else:
            counts["unstructured_event"] = (
                counts.get("unstructured_event", 0) + 1
            )

    return len(conversation), counts


def update_counter(counter_key, delta):
    """Update a counter avoiding negative values."""
    current_value = st.session_state.get(counter_key, 0)

    try:
        current_value = int(current_value)
    except (TypeError, ValueError):
        current_value = 0

    st.session_state[counter_key] = max(
        0,
        current_value + delta,
    )

def render_world_image(world_id):
    """Show the building plan image."""

    reference_image = WORLD_IMAGES.get(str(world_id))

    if reference_image and os.path.isfile(reference_image):
        from pathlib import Path
        import base64

        image_bytes = Path(reference_image).read_bytes()
        image_base64 = base64.b64encode(image_bytes).decode("utf-8")

        if world_id == "world_2":
            width = 1500
        else:
            width = 700

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

        st.caption(f"Building plan — {world_id}")

    else:
        st.warning(
            f"Building plan not found for {world_id}."
        )


# ---------- Loading the dataset ----------

st.title("LLM Conversation Reviewer")
st.caption(
    "Check the conversations and manually annotate errrors"
)

if not DATASET_FILE.is_file():
    st.error(
        f"File not found: {DATASET_FILE}"
    )
    st.stop()

try:
    with DATASET_FILE.open("r", encoding="utf-8") as f:
        dataset = json.load(f)

except (json.JSONDecodeError, OSError) as exc:
    st.error(f"Impossible to load the dataset: {exc}")
    st.stop()

if not isinstance(dataset, dict) or not dataset:
    st.error(
        "JSON file must coontain an object with a modality."
    )
    st.stop()


# ---------- Loading evaluations and error categories ----------

evaluations = read_json(EVALUATIONS_FILE, {})

if not isinstance(evaluations, dict):
    evaluations = {}

categories = read_json(CATEGORIES_FILE, DEFAULT_CATEGORIES)

if isinstance(categories, list):
    categories = {
        str(category): []
        for category in categories
        if isinstance(category, str) and category.strip()
    }

if not isinstance(categories, dict):
    categories = DEFAULT_CATEGORIES.copy()

normalized_categories = {}

for macro, subcategories in categories.items():
    if not isinstance(macro, str) or not macro.strip():
        continue

    if not isinstance(subcategories, list):
        continue

    normalized_categories[macro.strip()] = list(dict.fromkeys(
        sub.strip()
        for sub in subcategories
        if isinstance(sub, str) and sub.strip()
    ))

categories = normalized_categories


# ---------- Choosing modality, world_id, model, trial and run ----------

st.subheader("Navigation window")

# Selezione modalità
available_modes = [
    mode_id
    for mode_id, mode_data in dataset.items()
    if isinstance(mode_data, dict)
]

if not available_modes:
    st.error("No valid modality in the considered dataset.")
    st.stop()

# Quattro menu sulla stessa riga
nav_selectors = st.columns([1.5, 0.6, 1, 0.8], gap="small")

with nav_selectors[0]:
    mode = st.selectbox(
        "Modality",
        available_modes,
        key="selected_mode",
    )

mode_data = dataset[mode]

available_numbers = [
    str(world_id)
    for world_id, word_data in mode_data.items()
    if isinstance(word_data, dict)
]

if not available_numbers:
    st.error(f"Modality {mode} doesnt' have valid numbers.")
    st.stop()

number_key = f"selected_number_{mode}"

if st.session_state.get(number_key) not in available_numbers:
    st.session_state[number_key] = available_numbers[0]

with nav_selectors[1]:
    world_id = st.selectbox(
        "world_id",
        available_numbers,
        key=number_key,
    )

word_data = mode_data[world_id]

available_models = [
    str(model_id)
    for model_id, model_data in word_data.items()
    if isinstance(model_data, dict)
]

if not available_models:
    st.error(f"No valid model for {mode} / {world_id}.")
    st.stop()

model_key = f"selected_model_{mode}_{world_id}"

if st.session_state.get(model_key) not in available_models:
    st.session_state[model_key] = available_models[0]

with nav_selectors[2]:
    model_id = st.selectbox(
        "Model",
        available_models,
        key=model_key,
    )

model_data = word_data[model_id]

available_trials = [
    str(trial_id)
    for trial_id, trial_data in model_data.items()
    if isinstance(trial_data, dict)
]

if not available_trials:
    st.warning(
        f"No valid trial for {mode} / {world_id} / {model_id}."
    )
    st.stop()

trial_key = f"selected_trial_{mode}_{world_id}_{model_id}"

if st.session_state.get(trial_key) not in available_trials:
    st.session_state[trial_key] = available_trials[0]

with nav_selectors[3]:
    trial_id = st.selectbox(
        "Trial",
        available_trials,
        key=trial_key,
    )

trial_data = model_data[trial_id]


valid_runs = {
    str(run_id): run
    for run_id, run in trial_data.items()
    if isinstance(run, dict)
    and isinstance(run.get("conversation"), list)
}

if not valid_runs:
    st.warning(
        f"Trial {trial_id} doesn't containe valid run "
        "with a list 'conversation'."
    )
    st.stop()

world_runs = list(valid_runs.keys())

# Unique index for each combination
# modality / world_id / model / trial.
nav_key = (
    f"run_index_{mode}_{world_id}_{model_id}_{trial_id}"
)

if nav_key not in st.session_state:
    st.session_state[nav_key] = 0

st.session_state[nav_key] = min(
    max(0, st.session_state[nav_key]),
    len(world_runs) - 1,
)

nav_left, nav_center, nav_right = st.columns([1, 2, 1])

with nav_left:
    if st.button(
        "← Previous",
        disabled=st.session_state[nav_key] == 0,
        use_container_width=True,
        key=f"prev_{mode}_{world_id}_{model_id}_{trial_id}",
    ):
        st.session_state[nav_key] -= 1
        st.rerun()

with nav_right:
    if st.button(
        "Next →",
        disabled=(
            st.session_state[nav_key] >= len(world_runs) - 1
        ),
        use_container_width=True,
        key=f"next_{mode}_{world_id}_{model_id}_{trial_id}",
    ):
        st.session_state[nav_key] += 1
        st.rerun()

run_id = world_runs[st.session_state[nav_key]]
run = valid_runs[run_id]

# ---------- Evaluation state ----------

current_evaluation = (
    evaluations
    .get(mode, {})
    .get(world_id, {})
    .get(model_id, {})
    .get(trial_id, {})
    .get(run_id)
)

is_evaluated = (
    isinstance(current_evaluation, dict)
    and "error_counts" in current_evaluation
)

if st.session_state.get("evaluation_saved_message"):
    st.success(st.session_state.pop("evaluation_saved_message"))

with nav_center:
    st.markdown(
        f"<div style='text-align:center'>"
        f"<b>{run_id}</b><br>"
        f"{st.session_state[nav_key] + 1} "
        f"di {len(world_runs)}"
        f"</div>",
        unsafe_allow_html=True,
    )

    if is_evaluated:
        st.success("🟢 Evaluated")
    else:
        st.warning("🟠 To be evaluated")

st.caption(
    f"Current selection: "
    f"{mode} / {world_id} / {model_id} / "
    f"{trial_id} / {run_id}"
)


# ---------- Evalaution identification ----------

# Each combination modality / world_id / model / trial / run
# has an independent evaluation.

mode_evaluations = evaluations.setdefault(mode, {})

number_evaluations = mode_evaluations.setdefault(
    world_id, {}
)

model_evaluations = number_evaluations.setdefault(
    model_id, {}
)

trial_evaluations = model_evaluations.setdefault(
    trial_id, {}
)

old_evaluation = trial_evaluations.get(run_id, {})

if not isinstance(old_evaluation, dict):
    old_evaluation = {}

old_counts = old_evaluation.get("error_counts", {})
old_notes = old_evaluation.get("notes", "")

if not isinstance(old_counts, dict):
    old_counts = {}

if not isinstance(old_notes, str):
    old_notes = ""


# ---------- Interface ----------

st.subheader("Reference image")
left, right = st.columns([1.65, 1], gap="large")

with left:

    render_world_image(world_id)

    st.subheader("Conversation")

    st.caption(
        f"Modality: {mode} · "
        f"Trial: {trial_id} · "
        f"World: {world_id} · "
        f"Run: {run_id} · "
        f"Model: {model_id}"
    )
    
    # Show sequence of sensors
    st.subheader(f"Sequence: {run.get('sensor_sequence', [])}")

    # Conversation container
    with st.container(height=1250, border=True):
        for index, event in enumerate(run["conversation"]):
            render_event(event, index)


with right:
    # ---------- Conversation statistics ----------

    total_messages, message_counts = count_message_types(
        run["conversation"]
    )

    st.markdown("#### Message statistics")

    sorted_counts = sorted(
        message_counts.items(),
        key=lambda item: item[0].lower(),
    )

    stats_columns = st.columns(
        len(sorted_counts) + 1,
        gap="small",
    )

    with stats_columns[0]:
        st.metric(
            label="Total",
            value=total_messages,
        )

    for column, (role, count) in zip(
        stats_columns[1:],
        sorted_counts,
    ):
        with column:
            st.metric(
                label=role,
                value=count,
            )

    st.divider()


    # ---------- JSON Editor of error categories ----------

    editor_key = "category_dictionary_editor"

    # Converte il dizionario Python in JSON formattato.
    if editor_key not in st.session_state:
        st.session_state[editor_key] = json.dumps(
            categories,
            ensure_ascii=False,
            indent=4,
        )

    st.markdown("#### Error categories structure")
    st.caption(
        "Edit the JSON below. You can add, "
        "rename, or delete macro-categories and subcategories."
    )

    editor_content = st_ace(
        value=st.session_state[editor_key],
        language="json",
        theme="github",
        key="category_json_editor",
        height=240,
        tab_size=4,
        wrap=True,
        show_gutter=True,
        auto_update=True,
        font_size=18,
    )

    if st.button(
        "Apply structure",
        key="apply_category_dictionary",
        use_container_width=True,
    ):
        try:
            if not editor_content or not editor_content.strip():
                raise ValueError("JSON structure cannot be empty.")

            parsed_categories = json.loads(editor_content)

            if not isinstance(parsed_categories, dict):
                raise ValueError(
                    "Main structure must be a JSON."
                )

            validated_categories = {}

            for macro, subcategories in parsed_categories.items():
                if not isinstance(macro, str) or not macro.strip():
                    raise ValueError(
                        "Ogni macrocategoria deve essere una stringa non vuota."
                    )

                if not isinstance(subcategories, list):
                    raise ValueError(
                        f'Le sottocategorie di "{macro}" devono essere una lista.'
                    )

                if not all(
                    isinstance(sub, str) and sub.strip()
                    for sub in subcategories
                ):
                    raise ValueError(
                        f'Le sottocategorie di "{macro}" '
                        "devono essere stringhe non vuote."
                    )

                validated_categories[macro.strip()] = list(
                    dict.fromkeys(
                        sub.strip() for sub in subcategories
                    )
                )

            categories = validated_categories

            write_json(CATEGORIES_FILE, categories)

            # Aggiorna il testo dell'editor con il JSON normalizzato.
            st.session_state[editor_key] = json.dumps(
                categories,
                ensure_ascii=False,
                indent=4,
            )

            st.success("Struttura delle categorie aggiornata.")
            st.rerun()

        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            st.error(f"JSON not valid: {exc}")


    # ---------- Valutazione gerarchica ----------

    st.subheader("Evaluation")

    st.caption(
        "Modify the dictionary, apply the structure, and"
        "use + and − to mark errors."
    )


    # ---------- Contatori per sottocategoria ----------

    macro_totals = {}
    error_counts = {}

    for macro, subcategories in categories.items():

        # old_counts è già definita nella sezione
        # Identificazione della valutazione.
        saved_macro = old_counts.get(macro, {})

        if isinstance(saved_macro, dict):
            saved_subcounts = saved_macro.get(
                "subcategories", {}
            )

            if not isinstance(saved_subcounts, dict):
                saved_subcounts = {}
        else:
            # Compatibilità con vecchie valutazioni non gerarchiche.
            saved_subcounts = {}

        with st.expander(f"**{macro}**", expanded=True):

            subcounts = {}

            if not subcategories:
                st.caption(
                    "No subcategories."
                    "Add one to the dictionary above."
                )

            for subcategory in subcategories:

                # La chiave distingue ogni run e ogni sottocategoria.
                counter_key = (
                    f"counter_{mode}_{world_id}_{model_id}_"
                    f"{trial_id}_{run_id}_{macro}_{subcategory}"
                )

                # Inizializza solo se il contatore non è già presente.
                if counter_key not in st.session_state:
                    try:
                        initial_value = int(
                            saved_subcounts.get(subcategory, 0)
                        )
                    except (TypeError, ValueError):
                        initial_value = 0

                    st.session_state[counter_key] = max(
                        0, initial_value
                    )

                # Colonne più larghe per rendere visibili i pulsanti.
                label_col, minus_col, value_col, plus_col = (
                    st.columns(
                        [3.5, 1, 1.2, 1],
                        gap="small",
                        vertical_alignment="center",
                    )
                )

                with label_col:
                    st.markdown(subcategory)

                with minus_col:
                    st.button(
                        "−",
                        key=f"{counter_key}_minus",
                        help="Remove an error",
                        use_container_width=True,
                        on_click=update_counter,
                        args=(counter_key, -1),
                    )

                with plus_col:
                    st.button(
                        "＋",
                        key=f"{counter_key}_plus",
                        help="Add an error",
                        use_container_width=True,
                        on_click=update_counter,
                        args=(counter_key, 1),
                    )

                with value_col:
                    st.markdown(
                        f"""
                        <div style="
                            display: flex;
                            align-items: center;
                            justify-content: center;
                            width: 100%;
                            height: 38px;
                            box-sizing: border-box;
                            text-align: center;
                            font-size: 30px;
                            font-weight: 600;
                            line-height: 1;
                            margin: 0;
                            padding: 0;
                        ">
                            {st.session_state[counter_key]}
                        </div>
                        """,
                        unsafe_allow_html=True,
                    )
                # Legge il valore aggiornato dallo stato Streamlit.
                subcounts[subcategory] = int(
                    st.session_state[counter_key]
                )

            # Totale automatico della macrocategoria.
            macro_total = sum(subcounts.values())

            macro_totals[macro] = macro_total

            error_counts[macro] = {
                "subcategories": subcounts,
                "total": int(macro_total),
            }

            st.divider()

            st.markdown(
                f"""
                <div style="
                    display: flex;
                    align-items: center;
                    justify-content: flex-end;
                    gap: 8px;
                    padding: 4px 0;
                    font-size: 15px;
                ">
                    <span>Totale categoria:</span>
                    <span style="
                        font-weight: 700;
                        font-size: 16px;
                    ">{macro_total}</span>
                </div>
                """,
                unsafe_allow_html=True,
            )

    # ---------- Totale complessivo ----------

    total_errors = sum(macro_totals.values())

    st.markdown(
        f"""
        <div style="
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 12px;
            width: 100%;
            padding: 8px 0;
        ">
            <span style="
                font-size: 18px;
                font-weight: 600;
                white-space: nowrap;
            ">
                Totale complessivo degli errori
            </span>
            <span style="
                font-size: 28px;
                font-weight: 700;
                line-height: 1;
            ">
                {total_errors}
            </span>
        </div>
        """,
        unsafe_allow_html=True,
    )
    # ---------- Note ----------

    notes = st.text_area(
        "Note / motivations",
        value=old_notes,
        height=120,
        key=(
            f"notes_{mode}_{world_id}_{model_id}_"
            f"{trial_id}_{run_id}"
        ),
    )

    # ---------- Salvataggio ----------

    if st.button(
        "Save evaluation",
        type="primary",
        use_container_width=True,
        key=(
            f"save_{mode}_{world_id}_{model_id}_"
            f"{trial_id}_{run_id}"
        ),
    ):
        total_messages, message_counts = count_message_types(
            run["conversation"]
        )

        trial_evaluations[run_id] = {
            "sensor_sequence": run.get("sensor_sequence", []),

            "total_messages": int(total_messages),

            "message_counts": {
                role: int(count)
                for role, count in message_counts.items()
            },

            "error_counts": error_counts,

            "macro_totals": {
                macro: int(total)
                for macro, total in macro_totals.items()
            },

            "total_errors": int(total_errors),
            "notes": notes,
        }

        write_json(EVALUATIONS_FILE, evaluations)
        write_json(CATEGORIES_FILE, categories)

        st.success(
            "Evaluation saved for "
            f"{mode} / {world_id} / {model_id} / "
            f"{trial_id} / {run_id}"
        )

        st.rerun()