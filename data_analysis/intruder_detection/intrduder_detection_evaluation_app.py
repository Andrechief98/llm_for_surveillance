import json
from pathlib import Path

import streamlit as st
import os


script_dir = os.path.dirname(__file__) #/home/[...]/llm_for_surveillance/llm_data_analysis/intruder_detection
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
    /* Testo generale dell'applicazione */
    .stApp {
        font-size: 18px;
    }

    /* Testo di paragrafi, etichette e descrizioni */
    .stApp p,
    .stApp label,
    .stApp .stMarkdown,
    .stApp .stCaption {
        font-size: 18px;
    }

    /* Titoli e intestazioni */
    .stApp h1 {
        font-size: 2.2rem;
    }

    .stApp h2 {
        font-size: 1.8rem;
    }

    .stApp h3 {
        font-size: 1.5rem;
    }

    /* Campi di testo e menu a tendina */
    .stApp input,
    .stApp textarea,
    .stApp [data-baseweb="select"] {
        font-size: 17px;
    }

    /* Pulsanti */
    .stApp button {
        font-size: 17px;
    }

    /* Testo dei blocchi di codice */
    .stApp code,
    .stApp pre {
        font-size: 15px;
    }

    /* Testo dei messaggi assistant e user */
    .stApp [data-testid="stChatMessage"] p {
        font-size: 19px;
        line-height: 1.6;
    }

    /* Migliora la leggibilità del Markdown nelle risposte */
    .stApp [data-testid="stChatMessage"] li {
        font-size: 19px;
        line-height: 1.6;
    }

    /* Titoli Markdown all'interno dei messaggi */
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
CATEGORIES_FILE = Path("error_categories.json")

DEFAULT_CATEGORIES = [
    "Errore fattuale",
    "Errore di pianificazione",
    "Uso errato dei tool",
    "Violazione dei vincoli di sicurezza",
    "Errore di comunicazione",
]


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
    """Formatta JSON validi; mostra gli altri contenuti come testo."""
    if not isinstance(value, str):
        return json.dumps(value, ensure_ascii=False, indent=2)

    try:
        parsed = json.loads(value)
        return json.dumps(parsed, ensure_ascii=False, indent=2)
    except (json.JSONDecodeError, TypeError):
        return value


def render_event(event, index):
    """Assistant a sinistra, user a destra, tool invariati."""

    if not isinstance(event, dict):
        with st.container(border=True):
            st.caption(f"📄 EVENTO {index + 1}")
            st.code(pretty_content(event))
        return

    for role, content in event.items():
        role_lower = role.lower()

        if role_lower == "assistant":
            # Messaggio assistant allineato a sinistra
            left, right = st.columns([1, 1])

            with left:
                with st.chat_message("assistant", avatar="🤖"):
                    st.markdown(str(content))

        elif role_lower == "user":
            # Messaggio user allineato a destra
            left, right = st.columns([1, 1])

            with right:
                with st.chat_message("user", avatar="👤"):
                    st.markdown(str(content))

        elif role_lower == "function_call":
            with st.container(border=True):
                st.markdown(
                    f"🔧 **FUNCTION CALL** · Evento {index + 1}"
                )
                with st.expander("Visualizza dettagli"):
                    st.code(pretty_content(content))

        elif role_lower == "ros_output":
            with st.container(border=True):
                st.markdown(
                    f"⚙️ **ROS OUTPUT** · Evento {index + 1}"
                )
                with st.expander("Visualizza risultato ROS"):
                    st.code(pretty_content(content))

        elif role_lower == "tool":
            with st.container(border=True):
                st.markdown(
                    f"🧰 **TOOL OUTPUT** · Evento {index + 1}"
                )
                with st.expander("Visualizza risultato"):
                    st.code(pretty_content(content))

        else:
            with st.container(border=True):
                st.markdown(
                    f"📄 **{role.upper()}** · Evento {index + 1}"
                )
                with st.expander("Visualizza contenuto"):
                    st.code(pretty_content(content))

def count_message_types(conversation):
    """Conta i messaggi totali e li raggruppa per tipologia di mittente."""
    counts = {}

    for event in conversation:
        if isinstance(event, dict):
            if not event:
                counts["evento_vuoto"] = counts.get("evento_vuoto", 0) + 1
                continue

            # Ogni chiave rappresenta una tipologia di mittente.
            # Un evento con più chiavi viene contato per ciascuna tipologia.
            for role in event:
                counts[role] = counts.get(role, 0) + 1
        else:
            counts["evento_non_strutturato"] = (
                counts.get("evento_non_strutturato", 0) + 1
            )

    return len(conversation), counts


# def render_world_image(world_id):
#     """Mostra l'immagine associata al world_id selezionato."""

#     reference_image = WORLD_IMAGES.get(str(world_id))

#     if reference_image and os.path.isfile(reference_image):
#         _, image_col, _ = st.columns([1, 2, 1])

#         with image_col:
#             st.image(
#                 reference_image,
#                 caption=f"Planimetria — World {world_id}",
#                 width=700,
#             )

#             # Slider
#             # image_width = st.slider(
#             #     "Dimensione immagine",
#             #     min_value=150,
#             #     max_value=700,
#             #     value=320,
#             #     step=25,
#             # )

#             # st.image(
#             #     reference_image,
#             #     caption=f"Planimetria — World {world_id}",
#             #     width=image_width,
#             # )
#     else:
#         st.warning(
#             f"Immagine non trovata per world_id {world_id}."
#         )

def render_world_image(world_id):
    """Mostra l'immagine centrata nella colonna."""

    reference_image = WORLD_IMAGES.get(str(world_id))

    if reference_image and os.path.isfile(reference_image):
        from pathlib import Path
        import base64

        image_bytes = Path(reference_image).read_bytes()
        image_base64 = base64.b64encode(image_bytes).decode("utf-8")

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
                        width: 700px;
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

        st.caption(f"Planimetria — World {world_id}")

    else:
        st.warning(
            f"Immagine non trovata per world_id {world_id}."
        )

# ---------- Caricamento dataset ----------

st.title("LLM Conversation Reviewer")
st.caption(
    "Visualizzazione delle tracce e annotazione manuale degli errori"
)

uploaded = st.file_uploader(
    "Carica il dataset JSON",
    type=["json"],
)

if uploaded is None:
    st.info("Carica il file JSON contenente le modalità HITL e AUTO.")
    st.stop()

try:
    dataset = json.load(uploaded)
except (json.JSONDecodeError, UnicodeDecodeError):
    st.error("Il file caricato non è un JSON valido.")
    st.stop()

if not isinstance(dataset, dict) or not dataset:
    st.error("Il JSON deve contenere un oggetto con le modalità.")
    st.stop()


# ---------- Caricamento valutazioni e categorie ----------

evaluations = read_json(EVALUATIONS_FILE, {})
categories = read_json(CATEGORIES_FILE, DEFAULT_CATEGORIES)

if not isinstance(evaluations, dict):
    evaluations = {}

if not isinstance(categories, list):
    categories = DEFAULT_CATEGORIES.copy()


# ---------- Selezione modalità, world_id, modello, trial e run ----------

st.subheader("Navigazione")

# Selezione modalità
available_modes = [
    mode_id
    for mode_id, mode_data in dataset.items()
    if isinstance(mode_data, dict)
]

if not available_modes:
    st.error("Il dataset non contiene modalità valide.")
    st.stop()

# Quattro menu sulla stessa riga
nav_selectors = st.columns([1.5, 0.6, 1, 0.8], gap="small")

with nav_selectors[0]:
    mode = st.selectbox(
        "Modalità sperimentale",
        available_modes,
        key="selected_mode",
    )

mode_data = dataset[mode]

available_numbers = [
    str(word_id)
    for word_id, word_data in mode_data.items()
    if isinstance(word_data, dict)
]

if not available_numbers:
    st.error(f"La modalità {mode} non contiene numeri validi.")
    st.stop()

number_key = f"selected_number_{mode}"

if st.session_state.get(number_key) not in available_numbers:
    st.session_state[number_key] = available_numbers[0]

with nav_selectors[1]:
    word_id = st.selectbox(
        "world_id",
        available_numbers,
        key=number_key,
    )

word_data = mode_data[word_id]

available_models = [
    str(model_id)
    for model_id, model_data in word_data.items()
    if isinstance(model_data, dict)
]

if not available_models:
    st.error(f"Nessun modello valido per {mode} / {word_id}.")
    st.stop()

model_key = f"selected_model_{mode}_{word_id}"

if st.session_state.get(model_key) not in available_models:
    st.session_state[model_key] = available_models[0]

with nav_selectors[2]:
    model_id = st.selectbox(
        "Modello",
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
        f"Nessun trial valido per {mode} / {word_id} / {model_id}."
    )
    st.stop()

trial_key = f"selected_trial_{mode}_{word_id}_{model_id}"

if st.session_state.get(trial_key) not in available_trials:
    st.session_state[trial_key] = available_trials[0]

with nav_selectors[3]:
    trial_id = st.selectbox(
        "Trial",
        available_trials,
        key=trial_key,
    )

trial_data = model_data[trial_id]

# 5. Selezione run
valid_runs = {
    str(run_id): run
    for run_id, run in trial_data.items()
    if isinstance(run, dict)
    and isinstance(run.get("conversation"), list)
}

if not valid_runs:
    st.warning(
        f"Il trial {trial_id} non contiene run valide "
        "con una lista 'conversation'."
    )
    st.stop()

world_runs = list(valid_runs.keys())

# Indice indipendente per ogni combinazione
# modalità / world_id / modello / trial.
nav_key = (
    f"run_index_{mode}_{word_id}_{model_id}_{trial_id}"
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
        "← Precedente",
        disabled=st.session_state[nav_key] == 0,
        use_container_width=True,
        key=f"prev_{mode}_{word_id}_{model_id}_{trial_id}",
    ):
        st.session_state[nav_key] -= 1
        st.rerun()

with nav_right:
    if st.button(
        "Successiva →",
        disabled=(
            st.session_state[nav_key] >= len(world_runs) - 1
        ),
        use_container_width=True,
        key=f"next_{mode}_{word_id}_{model_id}_{trial_id}",
    ):
        st.session_state[nav_key] += 1
        st.rerun()

run_id = world_runs[st.session_state[nav_key]]
run = valid_runs[run_id]

with nav_center:
    st.markdown(
        f"<div style='text-align:center'>"
        f"<b>{run_id}</b><br>"
        f"{st.session_state[nav_key] + 1} "
        f"di {len(world_runs)}"
        f"</div>",
        unsafe_allow_html=True,
    )

st.caption(
    f"Selezione corrente: "
    f"{mode} / {word_id} / {model_id} / "
    f"{trial_id} / {run_id}"
)


# ---------- Identificazione della valutazione ----------

# Ogni combinazione modalità / world_id / modello / trial / run
# ha una valutazione indipendente.

mode_evaluations = evaluations.setdefault(mode, {})

number_evaluations = mode_evaluations.setdefault(
    word_id, {}
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


# ---------- Interfaccia affiancata ----------

st.subheader("Immagine di riferimento")
left, right = st.columns([1.65, 1], gap="large")

with left:
    # Immagine associata al world_id selezionato
    render_world_image(word_id)

    st.subheader("Conversazione")

    st.caption(
        f"Modalità: {mode} · "
        f"Trial: {trial_id} · "
        f"World: {run.get('world_id', 'N/D')} · "
        f"Run: {run_id} · "
        f"Modello: {run.get('model', 'N/D')}"
    )

    with st.container(height=650, border=True):
        with st.expander("Metadati della run"):
            metadata = {
                key: value
                for key, value in run.items()
                if key != "conversation"
            }
            st.json(metadata)

        for index, event in enumerate(run["conversation"]):
            render_event(event, index)


with right:
    # ---------- Statistiche conversazione ----------

    total_messages, message_counts = count_message_types(
        run["conversation"]
    )

    st.markdown("#### Statistiche messaggi")
    st.caption(f"Totale: {total_messages}")

    sorted_counts = sorted(
        message_counts.items(),
        key=lambda item: item[0].lower(),
    )

    columns_per_row = 3

    for start in range(0, len(sorted_counts), columns_per_row):
        row_counts = sorted_counts[start:start + columns_per_row]
        stats_columns = st.columns(columns_per_row, gap="small")

        for column, (role, count) in zip(stats_columns, row_counts):
            with column:
                st.metric(
                    label=role,
                    value=count,
                    label_visibility="visible",
                )

    st.divider()

    st.subheader("Valutazione")

    st.caption(
        "Registra il numero di errori per ciascuna categoria. "
        "Usa le note per motivare il giudizio."
    )

    # Editor delle categorie personalizzabili.
    categories_text = st.text_area(
        "Tipologie di errore (una per riga)",
        value="\n".join(str(c) for c in categories),
        height=130,
        key="category_editor",
    )

    current_categories = list(dict.fromkeys(
        line.strip()
        for line in categories_text.splitlines()
        if line.strip()
    ))

    counts = {}

    for category in current_categories:
        widget_key = (
            f"error_{mode}_{word_id}_{model_id}_"
            f"{trial_id}_{run_id}_{category}"
        )

        try:
            previous_value = int(old_counts.get(category, 0))
        except (TypeError, ValueError):
            previous_value = 0

        previous_value = max(0, previous_value)

        counts[category] = st.number_input(
            category,
            min_value=0,
            step=1,
            value=previous_value,
            key=widget_key,
        )

    total_errors = sum(counts.values())

    st.metric("Totale errori", total_errors)

    notes = st.text_area(
        "Note / motivazione",
        value=old_notes,
        height=120,
        key=(
            f"notes_{mode}_{word_id}_{model_id}_"
            f"{trial_id}_{run_id}"
        ),
    )

    if st.button(
        "Salva valutazione",
        type="primary",
        use_container_width=True,
    ):
        # Calcola le statistiche della conversazione
        total_messages, message_counts = count_message_types(
            run["conversation"]
        )

        trial_evaluations[run_id] = {
            "sensor_sequence": run.get("sensor_sequence", []),

            # Statistiche della conversazione
            "total_messages": int(total_messages),
            "message_counts": {
                authority: int(count)
                for authority, count in message_counts.items()
            },

            # Valutazione degli errori
            "error_counts": {
                category: int(count)
                for category, count in counts.items()
            },
            "total_errors": int(total_errors),
            "notes": notes,
        }

        write_json(EVALUATIONS_FILE, evaluations)
        write_json(CATEGORIES_FILE, current_categories)

        st.success(
            "Valutazione salvata per "
            f"{mode} / {word_id} / {model_id} / "
            f"{trial_id} / {run_id}"
        )