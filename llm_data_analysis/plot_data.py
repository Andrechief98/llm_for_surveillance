import json
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

# === CONFIGURA IL PERCORSO AL FILE JSON ===
json_path = "./llm_data_analysis/data_1.json"

def compute_SR_and_EFF_metrics(data):
    results = {}

    for experiment_type in data.keys():
        metrics = []

        for task in data[experiment_type]:
            seq = task["sensors_sequence"]
            trials = task["trials"]

            SUC_values = [] 
            UA_values = []
            NA_values = []


            for trial in trials:
                SUC_values.append(trial["SUC"])
                UA_values.append(trial["UA"])
                NA_values.append(trial["NA"])


            SR = sum(SUC_values) / len(trials)
            Eff = sum([1 - ua/na for ua,na in zip(UA_values,NA_values)]) / len(trials)

            metrics.append(
                {
                    "sequence":seq,
                    "SR":round(SR,2), 
                    "Eff" :round(Eff,2)
                }
            )
        results[experiment_type] = metrics
        print(results)

    return results


def extract_metrics(data):

    # AUTONOMOUS
    hall_dict_aut = {
        
    }

    error_plan_dict_aut = {
        
    }

    # MITL
    hall_dict_mitl = {
        
    }

    error_plan_dict_mitl = {
        
    }

    hall_corr_dict_mitl = {
        
    }

    error_plan_corr_dict_mitl = {
        
    }


    tot_hall_aut = 0
    tot_err_plan_aut = 0

    tot_hall_mitl = 0
    tot_err_plan_mitl = 0
    tot_hall_corr_mitl = 0
    tot_err_plan_corr_mitl = 0

    for experiment_type in data.keys():

        if "autonomous" in experiment_type:

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

                    for err_pl in trial["Annotations"]["ERROR PLAN"]:
                        keys_list = err_pl.keys()

                        for key in keys_list:
                            tot_err_plan_aut = tot_err_plan_aut + err_pl[key]
                            if key in error_plan_dict_aut.keys():
                                error_plan_dict_aut[key].append(err_pl[key])
                            else:
                                error_plan_dict_aut[key] = [err_pl[key]]

        elif "mitl" in experiment_type:

             for task in data[experiment_type]:
                trials = task["trials"]

                for trial in trials:

                    for hall in trial["Annotations"]["HALLUCINATION"]:
                        keys_list = hall.keys()

                        for key in keys_list:
                            tot_hall_mitl = tot_hall_mitl + hall[key]
                            if key in hall_dict_mitl.keys():
                                hall_dict_mitl[key].append(hall[key])
                            else:
                                hall_dict_mitl[key] = [hall[key]]

                    for err_pl in trial["Annotations"]["ERROR PLAN"]:
                        keys_list = err_pl.keys()

                        for key in keys_list:
                            tot_err_plan_mitl = tot_err_plan_mitl + err_pl[key]
                            if key in error_plan_dict_mitl.keys():
                                error_plan_dict_mitl[key].append(err_pl[key])
                            else:
                                error_plan_dict_mitl[key] = [err_pl[key]]

                    for hall_corr in trial["Annotations"]["HALLUCINATION CORRECTION"]:
                        keys_list = hall_corr.keys()

                        for key in keys_list:
                            tot_hall_corr_mitl = tot_hall_corr_mitl + hall_corr[key]
                            if key in hall_corr_dict_mitl.keys():
                                hall_corr_dict_mitl[key].append(hall_corr[key])
                            else:
                                hall_corr_dict_mitl[key] = [hall_corr[key]]
                    
                    for err_pl_corr in trial["Annotations"]["ERROR PLAN CORRECTION"]:
                        keys_list = err_pl_corr.keys()

                        for key in keys_list:
                            tot_err_plan_corr_mitl = tot_err_plan_corr_mitl + err_pl_corr[key]
                            if key in error_plan_corr_dict_mitl.keys():
                                error_plan_corr_dict_mitl[key].append(err_pl_corr[key])
                            else: 
                                
                                error_plan_corr_dict_mitl[key] = [err_pl_corr[key]]

        else:
            print("Wrong experiment type")
            return None

    # print(hall_dict_aut)
    # print(tot_hall_aut)

    # print(error_plan_dict_aut)
    # print(tot_err_plan_aut)

    # print(hall_dict_mitl)
    # print(tot_hall_mitl)

    # print(error_plan_dict_mitl)
    # print(tot_err_plan_mitl)

    # print(hall_corr_dict_mitl)    
    # print(tot_hall_corr_mitl)

    # print(error_plan_corr_dict_mitl)
    # print(tot_err_plan_corr_mitl)



    return hall_dict_aut, error_plan_dict_aut, tot_hall_aut, tot_err_plan_aut, hall_dict_mitl, error_plan_dict_mitl, tot_hall_mitl, tot_err_plan_mitl, hall_corr_dict_mitl, error_plan_corr_dict_mitl, tot_hall_corr_mitl, tot_err_plan_corr_mitl

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


def plot_stacked_bar(data_dict, tot, title):
    labels = list(data_dict.keys())
    # print(labels)
    values = list(data_dict.values())
    # print(labels)
    summed_values = []

    for value in values:
        summed_values.append(sum(value)/tot)

    fig, ax = plt.subplots(figsize=(10, 1.5))
    left = 0
    for i, (label, value) in enumerate(zip(labels, summed_values)):
        ax.barh(0, value, left=left, label=label)
        left += value

    ax.set_title(title)
    ax.set_yticks([])
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.show()


def plot_stacked_bar_correction(data_dict, data_dict_corr, title):
    print("##################")
    labels = list(data_dict.keys())
    labels_corr = list(data_dict_corr.keys())
    print(labels)
    print(labels_corr)

    for label in labels:
        data_dict[label] = sum(data_dict[label])
    
    for label in labels_corr:
        data_dict_corr[label] = sum(data_dict_corr[label])
    
    print(data_dict)
    print(data_dict_corr)



    x = np.arange(len(labels))

    # Plot barra piena (errore totale)
    plt.bar(x, data_dict.values(), label=labels, color='skyblue')

    # Plot barra tratteggiata sovrapposta (porzione corretta)
    plt.bar(x, data_dict.values(), label='Corretti da umano',
            color='none', edgecolor='black', hatch='///')

    # Etichette
    plt.xticks(x, labels)
    plt.ylabel('Percentuale (%)')
    plt.title('Fonti di errore LLM con correzioni umane sovrapposte')
    plt.legend()

    plt.tight_layout()
    plt.show()
    




if __name__ == '__main__':

    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    # metrics_results = compute_SR_and_EFF_metrics(json_data)

    # # Plot 2 grafici a barre per SR ed Efficienza
    # for experiment_type, metrics in metrics_results.items():
    #     plot_metrics(metrics, experiment_type)


    hall_dict_aut, error_plan_dict_aut, tot_hall_aut, tot_err_plan_aut, hall_dict_mitl, error_plan_dict_mitl, tot_hall_mitl, tot_err_plan_mitl, hall_corr_dict_mitl, error_plan_corr_dict_mitl, tot_hall_corr_mitl, tot_err_plan_corr_mitl = extract_metrics(json_data)


    # HALLUCINATIONS - AUTONOMOUS
    # plot_stacked_bar(hall_dict_aut, tot_hall_aut, "Source of Hallucinations - Autonomous")
    # # ERROR PLAN - AUTONOMOUS
    # plot_stacked_bar(error_plan_dict_aut, tot_err_plan_aut, "Source of Error Plans - Autonomous")

    # # HALLUCINATIONS - MITL
    # plot_stacked_bar(hall_dict_mitl, tot_hall_mitl, "Source of Hallucinations - MITL")
    # # ERROR PLAN - MITL
    # plot_stacked_bar(error_plan_dict_mitl, tot_err_plan_mitl, "Source of Error Plans - MITL")

    # HALLUCINATIONS CORRECTION- MITL
    plot_stacked_bar_correction(hall_dict_mitl, hall_corr_dict_mitl, "Source of Hallucinations - MITL")
    # ERROR PLAN CORRECTION- MITL
    plot_stacked_bar_correction(error_plan_dict_mitl, error_plan_corr_dict_mitl, "Source of Error Plans - MITL")



