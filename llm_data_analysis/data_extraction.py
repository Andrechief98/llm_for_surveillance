import pandas as pd
import json
import re

def parse_annotations(text):
    result = {
        "HALLUCINATION": [],
        "PLANNING ERROR": [],
        "ACTION ERROR": [],
        "FAIL": [],
        "HALLUCINATION CORRECTION": [],
        "PLANNING ERROR CORRECTION": []
    }

    if pd.isna(text):
        return result

    # Dividi il testo in blocchi in base ai tag

    if isinstance(text, str):
        blocks = re.split(r'\[(HALLUCINATION|PLANNING ERROR|ACTION ERROR|FAIL|HALLUCINATION CORRECTION|PLANNING ERROR CORRECTION)\]', text)
    else:
        print(f"type(text): {type(text)}")
        print(f"text: {text}")
        raise TypeError(f"Expected string for `text`, but got {type(text).__name__}")
        
    for i in range(1, len(blocks), 2):
        tag = blocks[i].strip()
        content = blocks[i + 1].strip()
        lines = content.split("\n")
        for line in lines:
            line = line.strip()
            print(line)
            if ":" in line:
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()
                if tag in ["HALLUCINATION", "PLANNING ERROR", "ACTION ERROR", "HALLUCINATION CORRECTION", "PLANNING ERROR CORRECTION"]:
                    try:
                        result[tag].append({key: int(value)})
                    except Exception as e:
                        print(f"Exception: {e}")
                elif tag == "FAIL":
                    result[tag].append({key: value})

                else:
                    print("######### ERROR IN THE EXTRACTION #########")
    return result

def extract_trials_from_sheet(df, split_on_comma_only=True, hitl=False):
    # Rimuove le prime due righe di intestazione
    df = df.iloc[2:].reset_index(drop=True)
    
    data = []

    for _, row in df.iterrows():
        if pd.isna(row[0]):
            continue

        raw_sequence = str(row[0]).strip()
        if split_on_comma_only:
            # Split solo su virgole
            sensors = [s.strip() for s in raw_sequence.split(',')]
        else:
            # Split su " - " per separare i sensori
            sensors = [s.strip() for s in raw_sequence.split(' - ')]

        trials = []
        for i in range(3):
            if hitl:
                offset = 1 + i * 9
                trial = {
                    "PE": row[offset],
                    "HF_PE": row[offset + 1],
                    "NA": row[offset + 2],
                    "AE": row[offset + 3],
                    "SUC": row[offset + 4],
                    "UA": row[offset + 5],
                    "TOT HAL": row[offset + 6],
                    "HF_HAL": row[offset + 7],
                    "Annotations": parse_annotations(row[offset + 8]),
                }

                trials.append(trial)
            else:
                offset = 1 + i * 7

                trial = {
                    "PE": row[offset],
                    "NA": row[offset + 1],
                    "AE": row[offset + 2],
                    "SUC": row[offset + 3],
                    "UA": row[offset + 4],
                    "TOT HAL": row[offset + 5],
                    "Annotations": parse_annotations(row[offset + 6]),
                }
                trials.append(trial)

        data.append({
            "sensors_sequence": sensors,
            "trials": trials
        })
    
    return data

# === MAIN ===

# Percorso al file Excel
file_path = "./llm_data_analysis/results_llm_surveillance_new.xlsx"

# Legge solo i fogli richiesti
sheet_names = ["LS-LLM", "LS-HITL", "UNC-LLM", "UNC-HITL"]
xls = pd.read_excel(file_path, sheet_name=sheet_names)

# Applica l'estrazione con logica diversa per ciascun foglio
result = {
    "LS-LLM": extract_trials_from_sheet(xls["LS-LLM"]),
    "UNC-LLM": extract_trials_from_sheet(xls["UNC-LLM"], split_on_comma_only=True),
    "LS-HITL": extract_trials_from_sheet(xls["LS-HITL"], hitl=True),
    "UNC-HITL": extract_trials_from_sheet(xls["UNC-HITL"], split_on_comma_only=True, hitl=True)
}

# Salva il JSON su file
with open("./llm_data_analysis/data.json", "w", encoding="utf-8") as f:
    json.dump(result, f, indent=2, ensure_ascii=False)

print("Estrazione completata. File salvato come 'data.json'")
