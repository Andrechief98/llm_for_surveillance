import pandas as pd
import json
import re

def parse_annotations(text):
    result = {
        "HALLUCINATION": [],
        "ERROR PLAN": [],
        "FAIL": [],
        "HALLUCINATION CORRECTION": [],
        "ERROR PLAN CORRECTION": []
    }

    if pd.isna(text):
        return result

    # Dividi il testo in blocchi in base ai tag
    blocks = re.split(r'\[(HALLUCINATION|ERROR PLAN|FAIL|HALLUCINATION CORRECTION|ERROR PLAN CORRECTION)\]', text)
    
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
                if tag in ["HALLUCINATION", "ERROR PLAN", "HALLUCINATION CORRECTION", "ERROR PLAN CORRECTION"]:
                    try:
                        result[tag].append({key: int(value)})
                    except Exception as e:
                        print(e)
                elif tag == "FAIL":
                    result[tag].append({key: value})
    return result

def extract_trials_from_sheet(df, split_on_comma_only=False):
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
            offset = 1 + i * 7
            trial = {
                "EP": row[offset],
                "NA": row[offset + 1],
                "EA": row[offset + 2],
                "SUC": row[offset + 3],
                "UA": row[offset + 4],
                "TOT ALL": row[offset + 5],
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
file_path = "./llm_data_analysis/results_llm_surveillance_1.xlsx"

# Legge solo i due fogli richiesti
sheet_names = ["Autonomous_FP&HI", "Autonomous_logical_sequence", "MITL_FP&HI", "MITL_logical_sequence"]
xls = pd.read_excel(file_path, sheet_name=sheet_names)

# Applica l'estrazione con logica diversa per ciascun foglio
result = {
    "autonomous_logical_sequence": extract_trials_from_sheet(xls["Autonomous_logical_sequence"]),
    "autonomous_FP&HI": extract_trials_from_sheet(xls["Autonomous_FP&HI"], split_on_comma_only=True),
    "mitl_logical_sequence": extract_trials_from_sheet(xls["MITL_logical_sequence"]),
    "mitl_FP&HI": extract_trials_from_sheet(xls["MITL_FP&HI"], split_on_comma_only=True)
}

# Salva il JSON su file
with open("./llm_data_analysis/data_1.json", "w", encoding="utf-8") as f:
    json.dump(result, f, indent=2, ensure_ascii=False)

print("Estrazione completata. File salvato come 'data.json'")
