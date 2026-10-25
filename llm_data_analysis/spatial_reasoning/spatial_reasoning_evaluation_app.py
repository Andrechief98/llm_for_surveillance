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
                st.markdown(f"🔧 **FUNCTION CALL** · Evento {index + 1}")

                function_name = "Funzione sconosciuta"
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

                st.markdown(f"**Funzione:** `{function_name}`")
                st.markdown("**Parametri:**")

                if isinstance(arguments, dict):
                    if arguments:
                        for key, value in arguments.items():
                            if isinstance(value, (dict, list)):
                                value = json.dumps(value, ensure_ascii=False)
                            st.markdown(f"- `{key}`: `{value}`")
                    else:
                        st.caption("Nessun parametro")
                else:
                    st.code(str(arguments), language="json")

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


        elif role_lower == "data_orchestrator":
            with st.container(border=True):
                st.markdown(f"🚨 **DATA ORCHESTRATOR** · Evento {index + 1}")

                # Estrae il JSON dello stato ambientale dal contenuto testuale
                match = re.search(
                    r"Current environment state:\s*(\{.*?\})\.\s*Propose",
                    content,
                    re.DOTALL
                )

                if match:
                    try:
                        # Il JSON contiene oggetti annidati: usa raw_decode
                        decoder = json.JSONDecoder()
                        state, _ = decoder.raw_decode(match.group(1))

                        robots = state.get("robots", {})
                        activated_sensors = state.get("activated_sensors", [])

                        st.markdown("#### 📡 Sensori attivi")

                        if activated_sensors:
                            st.markdown(
                                " · ".join(
                                    f"`{sensor}`" for sensor in activated_sensors
                                )
                            )
                        else:
                            st.caption("Nessun sensore attivo.")

                        st.markdown("#### 🤖 Robot")

                        if robots:
                            for robot_name, robot_info in robots.items():
                                location = robot_info.get("current_area", "N/D")
                                status = robot_info.get("status", "N/D")

                                st.markdown(
                                    f"- **{robot_name}** · "
                                    f"📍 Area: `{location}` · "
                                    f"Stato: {status}"
                                )
                        else:
                            st.caption("Nessun robot presente.")


                    except (json.JSONDecodeError, TypeError):
                        st.warning("Impossibile interpretare lo stato ambientale.")
                else:
                    st.warning("Stato ambientale non trovato nel messaggio.")

                # Mostra anche il testo originale, senza menu a tendina
                # st.markdown("#### 📝 Messaggio originale")
                # st.write(content)

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


def update_counter(counter_key, delta):
    """Aggiorna un contatore senza consentire valori negativi."""
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
    """Mostra l'immagine centrata nella colonna."""

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

        st.caption(f"Planimetria — {world_id}")

    else:
        st.warning(
            f"Immagine non trovata per {world_id}."
        )

# ---------- Caricamento dataset tramite browsing delle cartelle ----------

# st.title("LLM Conversation Reviewer")
# st.caption(
#     "Visualizzazione delle tracce e annotazione manuale degli errori"
# )

# uploaded = st.file_uploader(
#     "Carica il dataset JSON",
#     type=["json"],
# )

# if uploaded is None:
#     st.info("Carica il file JSON contenente le modalità HITL e AUTO.")
#     st.stop()

# try:
#     dataset = json.load(uploaded)
# except (json.JSONDecodeError, UnicodeDecodeError):
#     st.error("Il file caricato non è un JSON valido.")
#     st.stop()

# if not isinstance(dataset, dict) or not dataset:
#     st.error("Il JSON deve contenere un oggetto con le modalità.")
#     st.stop()


# ---------- Caricamento automatico dataset ----------

st.title("LLM Conversation Reviewer")
st.caption(
    "Visualizzazione delle tracce e annotazione manuale degli errori"
)

if not DATASET_FILE.is_file():
    st.error(
        f"File degli esperimenti non trovato: {DATASET_FILE}"
    )
    st.stop()

try:
    with DATASET_FILE.open("r", encoding="utf-8") as f:
        dataset = json.load(f)

except (json.JSONDecodeError, OSError) as exc:
    st.error(f"Impossibile caricare il dataset: {exc}")
    st.stop()

if not isinstance(dataset, dict) or not dataset:
    st.error(
        "Il JSON deve contenere un oggetto con le modalità."
    )
    st.stop()


# ---------- Caricamento valutazioni e categorie ----------

evaluations = read_json(EVALUATIONS_FILE, {})

if not isinstance(evaluations, dict):
    evaluations = {}

categories = read_json(CATEGORIES_FILE, DEFAULT_CATEGORIES)

# Compatibilità con il vecchio formato a lista.
if isinstance(categories, list):
    categories = {
        str(category): []
        for category in categories
        if isinstance(category, str) and category.strip()
    }

if not isinstance(categories, dict):
    categories = DEFAULT_CATEGORIES.copy()

# Normalizzazione del dizionario.
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
    str(world_id)
    for world_id, word_data in mode_data.items()
    if isinstance(word_data, dict)
]

if not available_numbers:
    st.error(f"La modalità {mode} non contiene numeri validi.")
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
    st.error(f"Nessun modello valido per {mode} / {world_id}.")
    st.stop()

model_key = f"selected_model_{mode}_{world_id}"

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
        f"Nessun trial valido per {mode} / {world_id} / {model_id}."
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
        "← Precedente",
        disabled=st.session_state[nav_key] == 0,
        use_container_width=True,
        key=f"prev_{mode}_{world_id}_{model_id}_{trial_id}",
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
        key=f"next_{mode}_{world_id}_{model_id}_{trial_id}",
    ):
        st.session_state[nav_key] += 1
        st.rerun()

run_id = world_runs[st.session_state[nav_key]]
run = valid_runs[run_id]

# ---------- Stato della valutazione ----------

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
        st.success("🟢 Già valutata")
    else:
        st.warning("🟠 Da valutare")

st.caption(
    f"Selezione corrente: "
    f"{mode} / {world_id} / {model_id} / "
    f"{trial_id} / {run_id}"
)


# ---------- Identificazione della valutazione ----------

# Ogni combinazione modalità / world_id / modello / trial / run
# ha una valutazione indipendente.

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


# ---------- Interfaccia affiancata ----------

st.subheader("Immagine di riferimento")
left, right = st.columns([1.65, 1], gap="large")

with left:
    # Immagine associata al world_id selezionato
    render_world_image(world_id)

    st.subheader("Conversazione")

    st.caption(
        f"Modalità: {mode} · "
        f"Trial: {trial_id} · "
        f"World: {world_id} · "
        f"Run: {run_id} · "
        f"Modello: {model_id}"
    )

    # with st.container(height=650, border=True):
    #     with st.expander("Metadati della run"):
    #         metadata = {
    #             key: value
    #             for key, value in run.items()
    #             if key != "conversation"
    #         }
    #         st.json(metadata)

    #     for index, event in enumerate(run["conversation"]):
    #         render_event(event, index)
    
    # Visualizza la sequenza sopra la conversazione
    st.subheader(f"Sequenza: {run.get('sensor_sequence', [])}")

    # Contenitore della conversazione
    with st.container(height=1250, border=True):
        for index, event in enumerate(run["conversation"]):
            render_event(event, index)


with right:
    # ---------- Statistiche conversazione ----------

    total_messages, message_counts = count_message_types(
        run["conversation"]
    )

    st.markdown("#### Statistiche messaggi")

    sorted_counts = sorted(
        message_counts.items(),
        key=lambda item: item[0].lower(),
    )

    # Totale e statistiche sulla stessa riga.
    stats_columns = st.columns(
        len(sorted_counts) + 1,
        gap="small",
    )

    with stats_columns[0]:
        st.metric(
            label="Totale",
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


    # ---------- Editor JSON delle categorie ----------

    editor_key = "category_dictionary_editor"

    # Converte il dizionario Python in JSON formattato.
    if editor_key not in st.session_state:
        st.session_state[editor_key] = json.dumps(
            categories,
            ensure_ascii=False,
            indent=4,
        )

    st.markdown("#### Struttura delle categorie")
    st.caption(
        "Modifica il JSON qui sotto. Puoi aggiungere, "
        "rinominare o eliminare macrocategorie e sottocategorie."
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
        "Applica struttura",
        key="apply_category_dictionary",
        use_container_width=True,
    ):
        try:
            if not editor_content or not editor_content.strip():
                raise ValueError("Il JSON non può essere vuoto.")

            parsed_categories = json.loads(editor_content)

            if not isinstance(parsed_categories, dict):
                raise ValueError(
                    "La struttura principale deve essere un oggetto JSON."
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
            st.error(f"JSON non valido: {exc}")


    # ---------- Valutazione gerarchica ----------

    st.subheader("Valutazione")

    st.caption(
        "Modifica il dizionario, applica la struttura e "
        "usa + e − per annotare gli errori."
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
                    "Nessuna sottocategoria. "
                    "Aggiungine una nel dizionario qui sopra."
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
                        help="Rimuovi un errore",
                        use_container_width=True,
                        on_click=update_counter,
                        args=(counter_key, -1),
                    )

                with plus_col:
                    st.button(
                        "＋",
                        key=f"{counter_key}_plus",
                        help="Aggiungi un errore",
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
        "Note / motivazione",
        value=old_notes,
        height=120,
        key=(
            f"notes_{mode}_{world_id}_{model_id}_"
            f"{trial_id}_{run_id}"
        ),
    )

    # ---------- Salvataggio ----------

    if st.button(
        "Salva valutazione",
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
            "Valutazione salvata per "
            f"{mode} / {world_id} / {model_id} / "
            f"{trial_id} / {run_id}"
        )

        st.rerun()