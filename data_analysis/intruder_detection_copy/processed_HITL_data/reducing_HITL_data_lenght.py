
import json
import os
import re

# It eliminates the useless keys in the HITL conversations to simplify the subsequent manual extraction of the inspection matrices

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

input_file = os.path.join(BASE_DIR, "../raw_data/experiments_HITL.json")
output_file = os.path.join(BASE_DIR, "experiments_HITL_cleared.json")

# Keys that determine deletion of the entire dictionary
keys_to_remove = {
    "reasoning",
    "file_search_call",
    "ROS_output"
}

# Keys to delete in "conversation"
keys_to_delete = {
    "semantic_RAG",
    "function_RAG",
    "image_info"
}

# Function call to keep
allowed_function_calls = {
    "send_robots_to_area",
    "get_camera_feed"
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


if __name__ == "__main__":

    # Reading input JSON file
    with open(input_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    data = keep_last_data_orchestrator(data)
    data = clean_conversation(data)

    formatted_json = format_json(data)

    # Saving the result in the output file
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(formatted_json)

    print(f"File saved in: {output_file}")
