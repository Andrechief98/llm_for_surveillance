
import json
import os
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

input_file = os.path.join(BASE_DIR, "prova.json")
output_file = os.path.join(BASE_DIR, "prova_clear_output.json")

# Chiavi che determinano l'eliminazione dell'intero dizionario
keys_to_remove = {
    "reasoning",
    "file_search_call",
    "ROS_output"
}

# Function call da mantenere
allowed_function_calls = {
    "send_robots_to_areas",
    "get_camera_feed"
}

# Chiavi da eliminare a livello del dizionario che contiene conversation
keys_to_delete = {
    "semantic_RAG",
    "function_RAG",
    "image_info"
}


def clean_data_orchestrator(text):

    if not isinstance(text, str):
        return text

    pattern = r'Current environment state:\s*(\{.*\})'

    match = re.search(pattern, text)

    if not match:
        return text

    try:
        # Estrae il JSON dello stato ambientale
        environment_json = match.group(1)

        # Decodifica il JSON in dictionary
        environment = json.loads(environment_json)

        # Mantiene esclusivamente activated_sensors
        cleaned_environment = {
            "activated_sensors": environment.get("activated_sensors", [])
        }

        # Rimuove l'intero environment state dal messaggio
        message = (
            text[:match.start()]
            + text[match.end():]
        ).strip()

        # Rimuove eventuali punti o spazi residui
        message = re.sub(r'\s+', ' ', message).strip()

        return {
            "message": message,
            "environment_state": cleaned_environment
        }

    except (json.JSONDecodeError, TypeError):
        return text





def clean_conversation(data):

    if isinstance(data, dict):

        # 1. Rimuove le chiavi contenenti "time"
        for key in list(data.keys()):
            if "time" in key.lower():
                del data[key]

        # 2. Elimina le chiavi specifiche
        for key in keys_to_delete:
            data.pop(key, None)

        # 3. Elimina interamente i dizionari
        # che contengono determinate chiavi
        if any(key in data for key in keys_to_remove):
            return None

        # 4. Controlla function_call
        if "function_call" in data:

            function_call = data["function_call"]

            if isinstance(function_call, str):

                first_line = function_call.splitlines()[0]
                parts = first_line.split("-", 1)

                function_name = (
                    parts[1].strip()
                    if len(parts) > 1 and parts[0].strip() == "name"
                    else None
                )

                if function_name not in allowed_function_calls:
                    return None

            else:
                return None

        # Trasforma data_orchestrator
        if "data_orchestrator" in data:
            data["data_orchestrator"] = clean_data_orchestrator(
                data["data_orchestrator"]
            )

        # 5. Continua ricorsivamente
        for key in list(data.keys()):

            cleaned_value = clean_conversation(data[key])

            if cleaned_value is None:
                del data[key]
            else:
                data[key] = cleaned_value

    elif isinstance(data, list):

        cleaned_list = []

        for item in data:

            cleaned_item = clean_conversation(item)

            if cleaned_item is not None:
                cleaned_list.append(cleaned_item)

        return cleaned_list

    return data


# Serializzazione personalizzata:
# ogni dizionario contenuto in conversation
# viene scritto su una singola riga.

def format_json(data, indent=4, level=0, parent_key=None):

    spaces = " " * (indent * level)
    next_spaces = " " * (indent * (level + 1))

    if isinstance(data, dict):

        if not data:
            return "{}"

        # Dizionario dentro conversation: una sola riga
        if parent_key == "conversation":
            return json.dumps(
                data,
                ensure_ascii=False,
                separators=(", ", ": ")
            )

        items = []

        for key, value in data.items():

            formatted_value = format_json(
                value,
                indent,
                level + 1,
                key
            )

            items.append(
                f'{next_spaces}{json.dumps(key)}: {formatted_value}'
            )

        return "{\n" + ",\n".join(items) + "\n" + spaces + "}"

    elif isinstance(data, list):

        if not data:
            return "[]"

        items = [
            format_json(item, indent, level + 1, parent_key)
            for item in data
        ]

        return (
            "[\n"
            + ",\n".join(next_spaces + item for item in items)
            + "\n" + spaces + "]"
        )

    else:
        return json.dumps(data, ensure_ascii=False)



def keep_last_data_orchestrator(data):

    if isinstance(data, dict):

        # Identifica una RUN
        if "conversation" in data and isinstance(data["conversation"], list):

            conversation = data["conversation"]

            # Individua l'ultimo data_orchestrator
            last_index = None

            for i, item in enumerate(conversation):

                if (
                    isinstance(item, dict)
                    and "data_orchestrator" in item
                ):
                    last_index = i

            # Elimina tutto ciò che precede l'ultimo data_orchestrator
            if last_index is not None:

                removed_count = last_index

                data["conversation"] = conversation[last_index:]

                print(
                    f"Conversation: {len(conversation)} elementi | "
                    f"Eliminati: {removed_count} | "
                    f"Mantenuti: {len(data['conversation'])}"
                )

        # Continua ricorsivamente nelle altre strutture
        for value in data.values():
            keep_last_data_orchestrator(value)

    elif isinstance(data, list):

        for item in data:
            keep_last_data_orchestrator(item)

    return data



# Lettura JSON
with open(input_file, "r", encoding="utf-8") as f:
    data = json.load(f)

# Mantiene soltanto l'ultimo data_orchestrator per ogni run
data = keep_last_data_orchestrator(data)

# Pulizia generale
data = clean_conversation(data)

# Formattazione personalizzata
formatted_json = format_json(data)

# Salvataggio
with open(output_file, "w", encoding="utf-8") as f:
    f.write(formatted_json)

print(f"File pulito salvato in: {output_file}")
