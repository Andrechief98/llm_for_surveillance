import json
from pathlib import Path

import streamlit as st

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


# ---------- Selezione modalità, trial e run ----------

st.subheader("Navigazione")

available_modes = [
    mode_id
    for mode_id, mode_data in dataset.items()
    if isinstance(mode_data, dict)
]

if not available_modes:
    st.error("Il dataset non contiene modalità valide.")
    st.stop()

mode = st.selectbox(
    "Modalità sperimentale",
    available_modes,
    key="selected_mode",
)

mode_data = dataset[mode]

available_trials = [
    str(trial_id)
    for trial_id, trial_data in mode_data.items()
    if isinstance(trial_data, dict)
]

if not available_trials:
    st.warning(f"La modalità {mode} non contiene trial validi.")
    st.stop()

trial_key = f"selected_trial_{mode}"

if st.session_state.get(trial_key) not in available_trials:
    st.session_state[trial_key] = available_trials[0]

trial_id = st.selectbox(
    "Trial",
    available_trials,
    key=trial_key,
)

trial_data = mode_data[trial_id]

valid_runs = {
    str(run_id): run
    for run_id, run in trial_data.items()
    if isinstance(run, dict)
    and isinstance(run.get("conversation"), list)
}

if not valid_runs:
    st.warning(
        f"Il trial {trial_id} della modalità {mode} "
        "non contiene run valide."
    )
    st.stop()

world_runs = list(valid_runs.keys())

# Indice indipendente per ciascuna combinazione modalità/trial.
nav_key = f"run_index_{mode}_{trial_id}"

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
        key=f"prev_{mode}_{trial_id}",
    ):
        st.session_state[nav_key] -= 1
        st.rerun()

with nav_right:
    if st.button(
        "Successiva →",
        disabled=st.session_state[nav_key] >= len(world_runs) - 1,
        use_container_width=True,
        key=f"next_{mode}_{trial_id}",
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
    f"Selezione corrente: {mode} / {trial_id} / {run_id}"
)


# ---------- Identificazione della valutazione ----------

# Ogni combinazione modalità/trial/run ha una valutazione indipendente.
mode_evaluations = evaluations.setdefault(mode, {})
trial_evaluations = mode_evaluations.setdefault(trial_id, {})

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

left, right = st.columns([1.65, 1], gap="large")

with left:
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
            f"error_{mode}_{trial_id}_{run_id}_{category}"
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
        key=f"notes_{mode}_{trial_id}_{run_id}",
    )

    if st.button(
        "Salva valutazione",
        type="primary",
        use_container_width=True,
    ):
        # Salva solo nella combinazione corrente.
        trial_evaluations[run_id] = {
            "world_id": run.get("world_id"),
            "model": run.get("model"),
            "sensor_sequence": run.get("sensor_sequence", []),
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
            f"{mode} / {trial_id} / {run_id}"
        )