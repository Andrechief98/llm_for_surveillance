import json
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

json_path = "./llm_data_analysis/data.json"

def compute_SR_and_EFF_metrics(data):
    results_for_trials = {}
    results_for_n_sens= {}
    planning_errors_for_n_sens = {}
    hallucinations_for_n_sens = {}

    for experiment_type in data.keys():
        metrics = []

        if experiment_type not in results_for_n_sens.keys():
            results_for_n_sens[experiment_type] = {
                "1" : {
                    "SR" : [],
                    "Eff": [],
                    "PE" : [],
                    "PE_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    },
                "2" : {
                    "SR" : [],
                    "Eff": [],
                    "PE" : [],
                    "PE_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    },
                "3" : {
                    "SR" : [],
                    "Eff": [],
                    "PE" : [],
                    "PE_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    }
            }

        if experiment_type not in planning_errors_for_n_sens.keys():
            planning_errors_for_n_sens[experiment_type] = {
                
            }

        if experiment_type not in hallucinations_for_n_sens.keys():
            hallucinations_for_n_sens[experiment_type] = {

            }

        for task in data[experiment_type]:
            seq = task["sensors_sequence"]
            trials = task["trials"]

            n_sens = len(seq)

            if n_sens not in planning_errors_for_n_sens[experiment_type].keys():
                planning_errors_for_n_sens[experiment_type][n_sens] = {
                    
                }

            if n_sens not in hallucinations_for_n_sens[experiment_type].keys():
                hallucinations_for_n_sens[experiment_type][n_sens] = {

                }


            SUC_values = [] 
            UA_values = []
            NA_values = []
            AE_values = []
            PE_values = []
            PE_corr_values = []
            HAL_values = []
            HAL_corr_values = []


            for trial in trials:
                SUC_values.append(trial["SUC"])
                UA_values.append(trial["UA"])
                NA_values.append(trial["NA"])
                AE_values.append(trial["AE"])
                PE_values.append(trial["PE"])
                HAL_values.append(trial["TOT HAL"])

                for error_plan_correct in trial["Annotations"]["PLANNING ERROR CORRECTION"]:
                    PE_corr_values.append(sum(error_plan_correct.values()))
                
                for error_plan_correct in trial["Annotations"]["HALLUCINATION CORRECTION"]:
                    HAL_corr_values.append(sum(error_plan_correct.values()))


                hallucinations = trial["Annotations"]["HALLUCINATION"]
                planning_errors = trial["Annotations"]["PLANNING ERROR"]

                
                for hallucination_dic in hallucinations:
                    if hallucination_dic:
                        hallucination_type = str(list(hallucination_dic.keys())[0])
                        if hallucination_type not in hallucinations_for_n_sens[experiment_type][n_sens].keys():
                            # hallucinations_for_n_sens[experiment_type][n_sens][hallucination_type] = {}
                            hallucinations_for_n_sens[experiment_type][n_sens][hallucination_type] = hallucination_dic[hallucination_type]
                        else:
                            hallucinations_for_n_sens[experiment_type][n_sens][hallucination_type] = hallucinations_for_n_sens[experiment_type][n_sens][hallucination_type] + hallucination_dic[hallucination_type]

                    else:
                        # Dictionary is empty
                        continue
                
                for planning_error_dic in planning_errors:
                    if planning_error_dic:
                        planning_error_type = str(list(planning_error_dic.keys())[0])
                        
                        if planning_error_type not in planning_errors_for_n_sens[experiment_type][n_sens].keys():
                            # planning_errors_for_n_sens[experiment_type][n_sens][planning_error_type] = {}
                            planning_errors_for_n_sens[experiment_type][n_sens][planning_error_type] = planning_error_dic[planning_error_type]
                        else:
                            planning_errors_for_n_sens[experiment_type][n_sens][planning_error_type] = planning_errors_for_n_sens[experiment_type][n_sens][planning_error_type] + planning_error_dic[planning_error_type]
                    else:
                        # Dictionary is empty
                        continue


            SR = sum(SUC_values) / len(trials)
            Eff = sum([1 - (ua+ea)/na for ua, ea, na in zip(UA_values, AE_values, NA_values)]) / len(trials)
            PE = sum(PE_values)
            PE_corr = sum(PE_corr_values)
            HAL = sum(HAL_values)
            HAL_corr = sum(HAL_corr_values)

            metrics.append(
                {
                    "sequence":seq,
                    "SR":round(SR,2), 
                    "Eff" :round(Eff,2)
                }
            )

            results_for_n_sens[experiment_type][str(len(seq))]["SR"].append(round(SR,2))
            results_for_n_sens[experiment_type][str(len(seq))]["Eff"].append(round(Eff,2))
            results_for_n_sens[experiment_type][str(len(seq))]["PE"].append(round(PE,2))
            results_for_n_sens[experiment_type][str(len(seq))]["PE_corr"].append(PE_corr)
            results_for_n_sens[experiment_type][str(len(seq))]["HAL"].append(round(HAL,2))
            results_for_n_sens[experiment_type][str(len(seq))]["HAL_corr"].append(HAL_corr)

        results_for_trials[experiment_type] = metrics
        # print(results_for_trials)

        for n_sens in results_for_n_sens[experiment_type]:
            results_for_n_sens[experiment_type][n_sens]["SR"] = round(sum(results_for_n_sens[experiment_type][n_sens]["SR"])/len(results_for_n_sens[experiment_type][n_sens]["SR"]),2)
            results_for_n_sens[experiment_type][n_sens]["Eff"] = round(sum(results_for_n_sens[experiment_type][n_sens]["Eff"])/len(results_for_n_sens[experiment_type][n_sens]["Eff"]),2)
            results_for_n_sens[experiment_type][n_sens]["PE"] = sum(results_for_n_sens[experiment_type][n_sens]["PE"])
            results_for_n_sens[experiment_type][n_sens]["PE_corr"] = sum(results_for_n_sens[experiment_type][n_sens]["PE_corr"])
            results_for_n_sens[experiment_type][n_sens]["HAL"] = sum(results_for_n_sens[experiment_type][n_sens]["HAL"])
            results_for_n_sens[experiment_type][n_sens]["HAL_corr"] = sum(results_for_n_sens[experiment_type][n_sens]["HAL_corr"])

    # print("Results for n_sens:")
    # print(results_for_n_sens)

    for test_type, sensors in results_for_n_sens.items():
        print(f"Test type: {test_type}")
        for sensor_count, metrics in sensors.items():
            print(f"  Sensors: {sensor_count}")
            print(f"""        SR: {metrics['SR']},
        Eff: {metrics['Eff']},
        PE: {metrics['PE']}, 
        PE_corr: {metrics['PE_corr']}, 
        HAL: {metrics['HAL']}, 
        HAL_corr: {metrics['HAL_corr']}""")
        print()

        
    # print("Planning errors for n_sens:")
    # print(planning_errors_for_n_sens)

    # print("Hallucination for n_sens:")
    # print(hallucinations_for_n_sens)

    return results_for_trials, results_for_n_sens, planning_errors_for_n_sens, hallucinations_for_n_sens



def extract_metrics(data):

    # AUTONOMOUS
    hall_dict_aut = {
        
    }

    error_plan_dict_aut = {
        
    }

    # HITL
    hall_dict_hitl = {
        
    }

    error_plan_dict_hitl = {
        
    }

    hall_corr_dict_hitl = {
        
    }

    error_plan_corr_dict_hitl = {
        
    }


    tot_hall_aut = 0
    tot_err_plan_aut = 0

    tot_hall_hitl = 0
    tot_err_plan_hitl = 0
    tot_hall_corr_hitl = 0
    tot_err_plan_corr_hitl = 0

    for experiment_type in data.keys():

        if "HITL" not in experiment_type:

            for task in data[experiment_type]:
                trials = task["trials"]

                for trial in trials:

                    for hall in trial["Annotations"]["HALLUCINATION"]:
                        keys_list = hall.keys()

                        for key in keys_list:
                            tot_hall_aut = tot_hall_aut + hall[key]
                            if key in hall_dict_aut.keys():
                                hall_dict_aut[key].append(hall[key])
                            else:
                                hall_dict_aut[key] = [hall[key]]

                    for err_pl in trial["Annotations"]["PLANNING ERROR"]:
                        keys_list = err_pl.keys()

                        for key in keys_list:
                            tot_err_plan_aut = tot_err_plan_aut + err_pl[key]
                            if key in error_plan_dict_aut.keys():
                                error_plan_dict_aut[key].append(err_pl[key])
                            else:
                                error_plan_dict_aut[key] = [err_pl[key]]

        elif "HITL" in experiment_type:

             for task in data[experiment_type]:
                trials = task["trials"]

                for trial in trials:

                    for hall in trial["Annotations"]["HALLUCINATION"]:
                        keys_list = hall.keys()

                        for key in keys_list:
                            tot_hall_hitl = tot_hall_hitl + hall[key]
                            if key in hall_dict_hitl.keys():
                                hall_dict_hitl[key].append(hall[key])
                            else:
                                hall_dict_hitl[key] = [hall[key]]

                    for err_pl in trial["Annotations"]["PLANNING ERROR"]:
                        keys_list = err_pl.keys()

                        for key in keys_list:
                            tot_err_plan_hitl = tot_err_plan_hitl + err_pl[key]
                            if key in error_plan_dict_hitl.keys():
                                error_plan_dict_hitl[key].append(err_pl[key])
                            else:
                                error_plan_dict_hitl[key] = [err_pl[key]]

                    for hall_corr in trial["Annotations"]["HALLUCINATION CORRECTION"]:
                        keys_list = hall_corr.keys()

                        for key in keys_list:
                            tot_hall_corr_hitl = tot_hall_corr_hitl + hall_corr[key]
                            if key in hall_corr_dict_hitl.keys():
                                hall_corr_dict_hitl[key].append(hall_corr[key])
                            else:
                                hall_corr_dict_hitl[key] = [hall_corr[key]]
                    
                    for err_pl_corr in trial["Annotations"]["PLANNING ERROR CORRECTION"]:
                        keys_list = err_pl_corr.keys()

                        for key in keys_list:
                            tot_err_plan_corr_hitl = tot_err_plan_corr_hitl + err_pl_corr[key]
                            if key in error_plan_corr_dict_hitl.keys():
                                error_plan_corr_dict_hitl[key].append(err_pl_corr[key])
                            else: 
                                
                                error_plan_corr_dict_hitl[key] = [err_pl_corr[key]]

        else:
            print("Wrong experiment type")
            return None

    
    print(f"# Autonomous test - Total hallucination: {tot_hall_aut}")
    for key,value in hall_dict_aut.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# Autonomous test - Total error plan: {tot_err_plan_aut}")
    for key,value in error_plan_dict_aut.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# HITL test - Total hallucination: {tot_hall_hitl}")
    for key,value in hall_dict_hitl.items():
        print(f"\t{key}: {sum(value)}")
    
    print(f"# HITL test - Total hallucination corrections: {tot_hall_corr_hitl}")
    for key,value in hall_corr_dict_hitl.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# HITL test - Total error plan: {tot_err_plan_hitl}")
    for key,value in error_plan_dict_hitl.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# HITL test - Total error plan corrections: {tot_err_plan_corr_hitl}")
    for key,value in error_plan_corr_dict_hitl.items():
        print(f"\t{key}: {sum(value)}")



    return hall_dict_aut, error_plan_dict_aut, tot_hall_aut, tot_err_plan_aut, hall_dict_hitl, error_plan_dict_hitl, tot_hall_hitl, tot_err_plan_hitl, hall_corr_dict_hitl, error_plan_corr_dict_hitl, tot_hall_corr_hitl, tot_err_plan_corr_hitl

def plot_metrics(metrics, experiment_type):
    tasks = [metric["sequence"] for metric in metrics]
    sr_values = [metric["SR"] for metric in metrics]
    eff_values = [metric["Eff"] for metric in metrics]

    # Spaziatura tra gruppi
    gap = 3
    group_size = 2

    # Calcolo posizioni con gap tra gruppi
    y_pos = []
    for i in range(len(tasks)):
        base = i * gap
        y_pos.extend([base, base + 1])

    fig, ax = plt.subplots(figsize=(10, len(tasks) * 0.6))
    bar_height = 0.8
    bar_width = 0.8

    sr_y = y_pos[::2]
    eff_y = y_pos[1::2]

    # Aggiungi barre SR
    ax.barh(sr_y, sr_values, height=bar_height, color='steelblue', label='Success Rate')

    # Aggiungi barre Eff
    ax.barh(eff_y, eff_values, height=bar_height, color='seagreen', label='Efficiency')

    # Etichette combinate ogni 2 righe
    yticks = [(sr + eff) / 2 for sr, eff in zip(sr_y, eff_y)]
    ax.set_yticks(yticks)
    ax.set_yticklabels(tasks)

    ax.set_xlim(0, 1)
    ax.set_xlabel("Metric Value")
    ax.set_title(f"SR and Efficiency - {experiment_type.replace('_', ' ').title()}")
    ax.legend(loc='lower right')

    plt.tight_layout()
    plt.show()


def create_subplots_n_sens(data):

    # Asse X: Number of sensors
    sensors = sorted(data['LS-LLM'].keys(), key=int)
    x = [int(s) for s in sensors]

    # --- Primo plot: Success Rate (SR) ---
    fig1, axs1 = plt.subplots(1, 2, figsize=(12, 5))

    # Subplot 1: logical_sequence
    axs1[0].set_title("Success Rate: logical_sequence")
    for prefix in ['LLM', 'HITL']:
        key = f"LS-{prefix}"
        y = [data[key][s]['SR'] for s in sensors]
        axs1[0].plot(x, y, marker='o', label=prefix)
    axs1[0].set_xlabel("Number of activated sensors")
    axs1[0].set_ylabel("SR")
    axs1[0].legend()
    axs1[0].set_ylim(0, 1.2)
    axs1[0].set_xticks(x)

    # Subplot 2: UNC
    axs1[1].set_title("Success Rate: UNC")
    for prefix in ['LLM', 'HITL']:
        key = f"UNC-{prefix}"
        y = [data[key][s]['SR'] for s in sensors]
        axs1[1].plot(x, y, marker='o', label=prefix)
    axs1[1].set_xlabel("Number of activated sensors")
    axs1[1].set_ylabel("SR")
    axs1[1].legend()
    axs1[1].set_ylim(0, 1.2)
    axs1[1].set_xticks(x)

    plt.tight_layout()
    plt.show()


    # --- Secondo plot: Efficiency (Eff) ---
    fig2, axs2 = plt.subplots(1, 2, figsize=(12, 5))

    # Subplot 1: logical_sequence
    axs2[0].set_title("Efficiency: logical_sequence")
    for prefix in ['LLM', 'HITL']:
        key = f"LS-{prefix}"
        y = [data[key][s]['Eff'] for s in sensors]
        axs2[0].plot(x, y, marker='o', label=prefix)
    axs2[0].set_xlabel("Number of activated sensors")
    axs2[0].set_ylabel("Efficiency")
    axs2[0].legend()
    axs2[0].set_ylim(0, 1.2)
    axs2[0].set_xticks(x)

    # Subplot 2: UNC
    axs2[1].set_title("Efficiency: UNC")
    for prefix in ['LLM', 'HITL']:
        key = f"UNC-{prefix}"
        y = [data[key][s]['Eff'] for s in sensors]
        axs2[1].plot(x, y, marker='o', label=prefix)
    axs2[1].set_xlabel("Number of activated sensors")
    axs2[1].set_ylabel("Efficiency")
    axs2[1].legend()
    axs2[1].set_ylim(0, 1.2)
    axs2[1].set_xticks(x)

    plt.tight_layout()
    plt.show()

    # Terzo plot: Error plot (PE)
    fig3, axs3 = plt.subplots(1, 2, figsize=(12, 5))

    width = 0.35  # larghezza delle barre

    # Subplot 1: logical_sequence
    axs3[0].set_title("Error Plan (PE): logical_sequence")
    y_auto_ls = [data['LS-LLM'][str(s)]['PE'] for s in x]
    y_hitl_ls = [data['LS-HITL'][str(s)]['PE'] for s in x]
    y_hitl_ls_corr = [data['LS-HITL'][str(s)]['PE_corr'] for s in x]

    # barre raggruppate
    axs3[0].bar([xi - width/2 for xi in x], y_auto_ls, width, label='Autonomous')
    axs3[0].bar([xi + width/2 for xi in x], y_hitl_ls, width, label='HITL')
    axs3[0].bar([xi + width/2 for xi in x], y_hitl_ls_corr, width, label='HITL - corrections', edgecolor='black', color='none', hatch='///')

    axs3[0].set_xlabel("Number of activated sensors")
    axs3[0].set_ylabel("PE")
    axs3[0].set_xticks(x)
    axs3[0].set_yticks([0,5,10,15,20])
    axs3[0].set_ylim(0, 20)
    axs3[0].legend()

    # Subplot 2: UNC
    axs3[1].set_title("Error Plan (PE): UNC")
    y_auto_fp = [data['UNC-LLM'][str(s)]['PE'] for s in x]
    y_hitl_fp = [data['UNC-HITL'][str(s)]['PE'] for s in x]
    y_hitl_fp_corr = [data['UNC-HITL'][str(s)]['PE_corr'] for s in x]

    axs3[1].bar([xi - width/2 for xi in x], y_auto_fp, width, label='Autonomous')
    axs3[1].bar([xi + width/2 for xi in x], y_hitl_fp, width, label='HITL')
    axs3[1].bar([xi + width/2 for xi in x], y_hitl_fp_corr, width, label='HITL - corrections', edgecolor='black', color='none', hatch='///')

    axs3[1].set_xlabel("Number of activated sensors")
    axs3[1].set_ylabel("PE")
    axs3[1].set_xticks(x)
    axs3[1].set_yticks([0,5,10,15,20])
    axs3[1].set_ylim(0, 20)
    axs3[1].legend()

    plt.tight_layout()
    plt.show()

    # Quarto plot: Hallucination plot (HALL)
    fig4, axs4 = plt.subplots(1, 2, figsize=(12, 5))

    width = 0.35  # larghezza delle barre

    # Subplot 1: logical_sequence
    axs4[0].set_title("Logical sequence")
    y_auto_ls = [data['LS-LLM'][str(s)]['HAL'] for s in x]
    y_hitl_ls = [data['LS-HITL'][str(s)]['HAL'] for s in x]
    y_hitl_ls_corr = [data['LS-HITL'][str(s)]['HAL_corr'] for s in x]

    # barre raggruppate
    axs4[0].bar([xi - width/2 for xi in x], y_auto_ls, width, label='Autonomous')
    axs4[0].bar([xi + width/2 for xi in x], y_hitl_ls, width, label='HITL')
    axs4[0].bar([xi + width/2 for xi in x], y_hitl_ls_corr, width, label='HITL - corrections', edgecolor='black', color='none', hatch='///')

    axs4[0].set_xlabel("Number of activated sensors")
    axs4[0].set_ylabel("HAL")
    axs4[0].set_xticks(x)
    axs4[0].set_yticks([0, 10, 20, 30, 40, 50])
    axs4[0].set_ylim(0, 50)
    axs4[0].legend()

    # Subplot 2: UNC
    axs4[1].set_title("UNC")
    y_auto_fp = [data['UNC-LLM'][str(s)]['HAL'] for s in x]
    y_hitl_fp = [data['UNC-HITL'][str(s)]['HAL'] for s in x]
    y_hitl_fp_corr = [data['UNC-HITL'][str(s)]['HAL_corr'] for s in x]

    axs4[1].bar([xi - width/2 for xi in x], y_auto_fp, width, label='Autonomous')
    axs4[1].bar([xi + width/2 for xi in x], y_hitl_fp, width, label='HITL')
    axs4[1].bar([xi + width/2 for xi in x], y_hitl_fp_corr, width, label='HITL - corrections', edgecolor='black', color='none', hatch='///')

    axs4[1].set_xlabel("Number of activated sensors")
    axs4[1].set_ylabel("HAL")
    axs4[1].set_xticks(x)
    axs4[1].set_yticks([0, 10, 20, 30, 40, 50])
    axs4[1].set_ylim(0, 50)
    axs4[1].legend()

    plt.tight_layout()
    plt.show()


def plot_stacked_bar_overall(data_dict, tot, title):
    print(data_dict)
    labels = list(data_dict.keys())
    # print(labels)
    values = list(data_dict.values())
    # print(labels)
    summed_values = []

    for value in values:
        summed_values.append(round(sum(value)/tot,4))

    print(f"{title} - Data:")

    fig, ax = plt.subplots(figsize=(10, 1.5))
    left = 0
    for i, (label, value) in enumerate(zip(labels, summed_values)):
        print(f"{label}")
        print(f"{value}")
        ax.barh(0, value, left=left, label=label)
        left += value

    ax.set_title(title)
    ax.set_yticks([])
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.show()


def plot_stacked_bar_for_n_sens(planning_errors, hallucination):

    # print("Planning errors (raw):")
    # print(planning_errors)

    experiments = ["LS", "UNC"]
    models = ["LLM", "HITL"]
    sensors = [1, 2, 3]

    for exp in experiments:

        # Stampare percentuali per l'esperimento (planning + hallucination)
        print(f"\n\n=== Esperimento: {exp} ===")

        # --- Planning: stampa percentuali ---
        print("\n-- Planning errors --")
        for model in models:
            key = f"{exp}-{model}"
            print(f"\nModello: {key}")
            model_map = planning_errors.get(key)

            for s in sensors:
                counts = model_map.get(s, {})
                total = sum(counts.values())

                if total == 0:
                    print(f"  Sensori = {s}: nessun errore (totale 0).")
                    continue
                print(f"  Sensori = {s} (totale errori = {total}):")

                # ordino per percentuale decrescente
                sorted_items = sorted(counts.items(), key=lambda kv: -kv[1])
                for et, cnt in sorted_items:
                    pct = (cnt / total) * 100.0
                    print(f"    - {et}: {pct:.2f}% ({cnt} occorrenze)")

        # --- Hallucination: stampa percentuali ---
        print("\n-- Hallucination --")
        for model in models:
            key = f"{exp}-{model}"
            print(f"\nModello: {key}")
            model_map = hallucination.get(key, {})
            for s in sensors:
                counts = model_map.get(s, {}) or {}
                total = sum(counts.values())
                if total == 0:
                    print(f"  Sensori = {s}: nessuna occorrenza (totale 0).")
                    continue
                print(f"  Sensori = {s} (totale occorrenze = {total}):")
                sorted_items = sorted(counts.items(), key=lambda kv: -kv[1])
                for et, cnt in sorted_items:
                    pct = (cnt / total) * 100.0
                    print(f"    - {et}: {pct:.2f}% ({cnt} occorrenze)")

        # --- Costruzione figure: una figura per esperimento con 2 subplot affiancati ---
        fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(14, 5))
        subplot_specs = [
            ("Planning errors", planning_errors, True),    # (title, source_dict, is_planning)
            ("Hallucination", hallucination, False)
        ]

        for ax, (title, source_dict, is_pl) in zip(axes, subplot_specs):
            # error types: semplice union su entrambi i modelli e tutti i sensori
            ets = set()
            for model in models:
                key = f"{exp}-{model}"
                for s in sensors:
                    ets.update((source_dict.get(key, {}).get(s, {}) or {}).keys())
            error_types = sorted(ets)
            if not error_types:
                ax.set_title(f"{exp} — {title} (nessun errore)")
                ax.set_xticks(range(len(sensors)))
                ax.set_xticklabels([str(s) for s in sensors])
                continue

            cmap = plt.get_cmap("tab20")
            colors = [cmap(i % 20) for i in range(len(error_types))]

            x = np.arange(len(sensors))
            width = 0.35
            x_llm = x - width/2
            x_hitl = x + width/2

            bottoms_llm = np.zeros(len(sensors))
            bottoms_hitl = np.zeros(len(sensors))

            for idx_et, et in enumerate(error_types):
                vals_llm = []
                vals_hitl = []
                # costruisco il vettore di percentuali per ogni sensore
                for s in sensors:
                    # LLM
                    cnt_llm = (source_dict.get(f"{exp}-LLM", {}).get(s, {}) or {}).get(et, 0)
                    total_llm = sum((source_dict.get(f"{exp}-LLM", {}).get(s, {}) or {}).values())
                    pct_llm = (cnt_llm / total_llm * 100.0) if total_llm > 0 else 0.0
                    vals_llm.append(pct_llm)
                    # HITL
                    cnt_hitl = (source_dict.get(f"{exp}-HITL", {}).get(s, {}) or {}).get(et, 0)
                    total_hitl = sum((source_dict.get(f"{exp}-HITL", {}).get(s, {}) or {}).values())
                    pct_hitl = (cnt_hitl / total_hitl * 100.0) if total_hitl > 0 else 0.0
                    vals_hitl.append(pct_hitl)

                vals_llm = np.array(vals_llm)
                vals_hitl = np.array(vals_hitl)

                ax.bar(x_llm, vals_llm, bottom=bottoms_llm, width=width, color=colors[idx_et])
                ax.bar(x_hitl, vals_hitl, bottom=bottoms_hitl, width=width, color=colors[idx_et])
                bottoms_llm += vals_llm
                bottoms_hitl += vals_hitl

            ax.set_xticks(x)
            ax.set_xticklabels([str(s) for s in sensors])
            ax.set_xlabel("Numero di sensori")
            ax.set_ylabel("Percentuale (%)")
            ax.set_title(f"{exp} — {title}")
            ax.set_ylim(0, 100)

            # legenda: tipi di errore
            handles = [Patch(facecolor=colors[i], label=error_types[i]) for i in range(len(error_types))]
            if handles:
                ax.legend(handles=handles, bbox_to_anchor=(1.02, 1), loc="upper left", fontsize="small")

            # piccolo reminder LLM/HITL
            ax.text(1.02, 0.45, "Barre: left=LLM, right=HITL", transform=ax.transAxes,
                    rotation=90, va="center", ha="left", fontsize=9)

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':

    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    
    hall_dict_aut, error_plan_dict_aut, tot_hall_aut, tot_err_plan_aut, hall_dict_hitl, error_plan_dict_hitl, tot_hall_hitl, tot_err_plan_hitl, hall_corr_dict_hitl, error_plan_corr_dict_hitl, tot_hall_corr_hitl, tot_err_plan_corr_hitl = extract_metrics(json_data)


    metrics_results_trials, metrics_results_n_sens, planning_errors_for_n_sens, hallucinations_for_n_sens = compute_SR_and_EFF_metrics(json_data)

    # SR and Eff per n_activated_sensors:
    create_subplots_n_sens(metrics_results_n_sens)

    # # Plot 2 grafici a barre per SR ed Efficienza
    # for experiment_type, metrics in metrics_results_trials.items():
    #     plot_metrics(metrics, experiment_type)


    # HALLUCINATIONS - AUTONOMOUS
    plot_stacked_bar_overall(hall_dict_aut, tot_hall_aut, "Source of Hallucinations - LLM")
    # PLANNING ERROR - AUTONOMOUS
    plot_stacked_bar_overall(error_plan_dict_aut, tot_err_plan_aut, "Source of Planning Errors - LLM")

    # HALLUCINATIONS - HITL
    plot_stacked_bar_overall(hall_dict_hitl, tot_hall_hitl, "Source of Hallucinations - HITL")
    # PLANNING ERROR - HITL
    plot_stacked_bar_overall(error_plan_dict_hitl, tot_err_plan_hitl, "Source of Planning Errors - HITL")

    
    plot_stacked_bar_for_n_sens(planning_errors_for_n_sens, hallucinations_for_n_sens)