import json
import matplotlib.pyplot as plt
import numpy as np

json_path = "./llm_data_analysis/data.json"

def compute_SR_and_EFF_metrics(data):
    results_for_trials = {}
    results_for_n_sens= {}

    for experiment_type in data.keys():
        metrics = []

        if experiment_type not in results_for_n_sens.keys():
            results_for_n_sens[experiment_type] = {
                "1" : {
                    "SR" : [],
                    "Eff": [],
                    "EP" : [],
                    "EP_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    },
                "2" : {
                    "SR" : [],
                    "Eff": [],
                    "EP" : [],
                    "EP_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    },
                "3" : {
                    "SR" : [],
                    "Eff": [],
                    "EP" : [],
                    "EP_corr" : [],
                    "HAL":[],
                    "HAL_corr": []
                    }
            }

        for task in data[experiment_type]:
            seq = task["sensors_sequence"]
            trials = task["trials"]

            SUC_values = [] 
            UA_values = []
            NA_values = []
            EA_values = []
            EP_values = []
            EP_corr_values = []
            HAL_values = []
            HAL_corr_values = []


            for trial in trials:
                SUC_values.append(trial["SUC"])
                UA_values.append(trial["UA"])
                NA_values.append(trial["NA"])
                EA_values.append(trial["EA"])
                EP_values.append(trial["EP"])
                HAL_values.append(trial["TOT HAL"])

                for error_plan_correct in trial["Annotations"]["ERROR PLAN CORRECTION"]:
                    EP_corr_values.append(sum(error_plan_correct.values()))
                
                for error_plan_correct in trial["Annotations"]["HALLUCINATION CORRECTION"]:
                    HAL_corr_values.append(sum(error_plan_correct.values()))


            SR = sum(SUC_values) / len(trials)
            Eff = sum([1 - (ua+ea)/na for ua, ea, na in zip(UA_values, EA_values, NA_values)]) / len(trials)
            EP = sum(EP_values)
            EP_corr = sum(EP_corr_values)
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
            results_for_n_sens[experiment_type][str(len(seq))]["EP"].append(round(EP,2))
            results_for_n_sens[experiment_type][str(len(seq))]["EP_corr"].append(EP_corr)
            results_for_n_sens[experiment_type][str(len(seq))]["HAL"].append(round(HAL,2))
            results_for_n_sens[experiment_type][str(len(seq))]["HAL_corr"].append(HAL_corr)

        results_for_trials[experiment_type] = metrics
        # print(results_for_trials)

        for n_sens in results_for_n_sens[experiment_type]:
            results_for_n_sens[experiment_type][n_sens]["SR"] = round(sum(results_for_n_sens[experiment_type][n_sens]["SR"])/len(results_for_n_sens[experiment_type][n_sens]["SR"]),2)
            results_for_n_sens[experiment_type][n_sens]["Eff"] = round(sum(results_for_n_sens[experiment_type][n_sens]["Eff"])/len(results_for_n_sens[experiment_type][n_sens]["Eff"]),2)
            results_for_n_sens[experiment_type][n_sens]["EP"] = sum(results_for_n_sens[experiment_type][n_sens]["EP"])
            results_for_n_sens[experiment_type][n_sens]["EP_corr"] = sum(results_for_n_sens[experiment_type][n_sens]["EP_corr"])
            results_for_n_sens[experiment_type][n_sens]["HAL"] = sum(results_for_n_sens[experiment_type][n_sens]["HAL"])
            results_for_n_sens[experiment_type][n_sens]["HAL_corr"] = sum(results_for_n_sens[experiment_type][n_sens]["HAL_corr"])

    print(results_for_n_sens)

        

    return results_for_trials, results_for_n_sens



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

    
    print(f"# Autonomous test - Total hallucination: {tot_hall_aut}")
    for key,value in hall_dict_aut.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# Autonomous test - Total error plan: {tot_err_plan_aut}")
    for key,value in error_plan_dict_aut.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# MITL test - Total hallucination: {tot_hall_mitl}")
    for key,value in hall_dict_mitl.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# MITL test - Total error plan: {tot_err_plan_mitl}")
    for key,value in error_plan_dict_mitl.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# MITL test + correction - Total hallucination: {tot_hall_corr_mitl}")
    for key,value in hall_corr_dict_mitl.items():
        print(f"\t{key}: {sum(value)}")

    print(f"# MITL test + correction - Total error plan: {tot_err_plan_corr_mitl}")
    for key,value in error_plan_corr_dict_mitl.items():
        print(f"\t{key}: {sum(value)}")



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


def create_subplots_n_sens(data):

    # Asse X: Number of sensors
    sensors = sorted(data['autonomous_logical_sequence'].keys(), key=int)
    x = [int(s) for s in sensors]

    # --- Primo plot: Success Rate (SR) ---
    fig1, axs1 = plt.subplots(1, 2, figsize=(12, 5))

    # Subplot 1: logical_sequence
    axs1[0].set_title("Success Rate: logical_sequence")
    for prefix in ['Autonomous', 'MITL']:
        key = f"{prefix.lower()}_logical_sequence"
        y = [data[key][s]['SR'] for s in sensors]
        axs1[0].plot(x, y, marker='o', label=prefix)
    axs1[0].set_xlabel("Number of activated sensors")
    axs1[0].set_ylabel("SR")
    axs1[0].legend()
    axs1[0].set_ylim(0, 1.2)
    axs1[0].set_xticks(x)

    # Subplot 2: FP&HI
    axs1[1].set_title("Success Rate: FP&HI")
    for prefix in ['Autonomous', 'MITL']:
        key = f"{prefix.lower()}_FP&HI"
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
    for prefix in ['Autonomous', 'MITL']:
        key = f"{prefix.lower()}_logical_sequence"
        y = [data[key][s]['Eff'] for s in sensors]
        axs2[0].plot(x, y, marker='o', label=prefix)
    axs2[0].set_xlabel("Number of activated sensors")
    axs2[0].set_ylabel("Efficiency")
    axs2[0].legend()
    axs2[0].set_ylim(0, 1.2)
    axs2[0].set_xticks(x)

    # Subplot 2: FP&HI
    axs2[1].set_title("Efficiency: FP&HI")
    for prefix in ['Autonomous', 'MITL']:
        key = f"{prefix.lower()}_FP&HI"
        y = [data[key][s]['Eff'] for s in sensors]
        axs2[1].plot(x, y, marker='o', label=prefix)
    axs2[1].set_xlabel("Number of activated sensors")
    axs2[1].set_ylabel("Efficiency")
    axs2[1].legend()
    axs2[1].set_ylim(0, 1.2)
    axs2[1].set_xticks(x)

    plt.tight_layout()
    plt.show()

    # Terzo plot: Error plot (EP)
    fig3, axs3 = plt.subplots(1, 2, figsize=(12, 5))

    width = 0.35  # larghezza delle barre

    # Subplot 1: logical_sequence
    axs3[0].set_title("Error Plan (EP): logical_sequence")
    y_auto_ls = [data['autonomous_logical_sequence'][str(s)]['EP'] for s in x]
    y_mitl_ls = [data['mitl_logical_sequence'][str(s)]['EP'] for s in x]
    y_mitl_ls_corr = [data['mitl_logical_sequence'][str(s)]['EP_corr'] for s in x]

    # barre raggruppate
    axs3[0].bar([xi - width/2 for xi in x], y_auto_ls, width, label='Autonomous')
    axs3[0].bar([xi + width/2 for xi in x], y_mitl_ls, width, label='MITL')
    axs3[0].bar([xi + width/2 for xi in x], y_mitl_ls_corr, width, label='MITL - corrections', edgecolor='black', color='none', hatch='///')

    axs3[0].set_xlabel("Number of activated sensors")
    axs3[0].set_ylabel("EP")
    axs3[0].set_xticks(x)
    axs3[0].set_yticks([0,5,10,15,20])
    axs3[0].set_ylim(0, 20)
    axs3[0].legend()

    # Subplot 2: FP&HI
    axs3[1].set_title("Error Plan (EP): FP&HI")
    y_auto_fp = [data['autonomous_FP&HI'][str(s)]['EP'] for s in x]
    y_mitl_fp = [data['mitl_FP&HI'][str(s)]['EP'] for s in x]
    y_mitl_fp_corr = [data['mitl_FP&HI'][str(s)]['EP_corr'] for s in x]

    axs3[1].bar([xi - width/2 for xi in x], y_auto_fp, width, label='Autonomous')
    axs3[1].bar([xi + width/2 for xi in x], y_mitl_fp, width, label='MITL')
    axs3[1].bar([xi + width/2 for xi in x], y_mitl_fp_corr, width, label='MITL - corrections', edgecolor='black', color='none', hatch='///')

    axs3[1].set_xlabel("Number of activated sensors")
    axs3[1].set_ylabel("EP")
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
    y_auto_ls = [data['autonomous_logical_sequence'][str(s)]['HAL'] for s in x]
    y_mitl_ls = [data['mitl_logical_sequence'][str(s)]['HAL'] for s in x]
    y_mitl_ls_corr = [data['mitl_logical_sequence'][str(s)]['HAL_corr'] for s in x]

    # barre raggruppate
    axs4[0].bar([xi - width/2 for xi in x], y_auto_ls, width, label='Autonomous')
    axs4[0].bar([xi + width/2 for xi in x], y_mitl_ls, width, label='MITL')
    axs4[0].bar([xi + width/2 for xi in x], y_mitl_ls_corr, width, label='MITL - corrections', edgecolor='black', color='none', hatch='///')

    axs4[0].set_xlabel("Number of activated sensors")
    axs4[0].set_ylabel("EP")
    axs4[0].set_xticks(x)
    axs4[0].set_yticks([0, 10, 20, 30, 40, 50])
    axs4[0].set_ylim(0, 50)
    axs4[0].legend()

    # Subplot 2: FP&HI
    axs4[1].set_title("FP&HI")
    y_auto_fp = [data['autonomous_FP&HI'][str(s)]['HAL'] for s in x]
    y_mitl_fp = [data['mitl_FP&HI'][str(s)]['HAL'] for s in x]
    y_mitl_fp_corr = [data['mitl_FP&HI'][str(s)]['HAL_corr'] for s in x]

    axs4[1].bar([xi - width/2 for xi in x], y_auto_fp, width, label='Autonomous')
    axs4[1].bar([xi + width/2 for xi in x], y_mitl_fp, width, label='MITL')
    axs4[1].bar([xi + width/2 for xi in x], y_mitl_fp_corr, width, label='MITL - corrections', edgecolor='black', color='none', hatch='///')

    axs4[1].set_xlabel("Number of activated sensors")
    axs4[1].set_ylabel("HAL")
    axs4[1].set_xticks(x)
    axs4[1].set_yticks([0, 10, 20, 30, 40, 50])
    axs4[1].set_ylim(0, 50)
    axs4[1].legend()

    plt.tight_layout()
    plt.show()


def plot_stacked_bar(data_dict, tot, title):
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


def plot_stacked_bar_correction(data_dict, data_dict_corr, title):
    
    # Estrai le etichette (chiavi) e calcola i valori sommati
    labels = list(data_dict.keys())
    # values = [sum(data_dict[k]) for k in labels]
    values = [1 for k in labels] # Percentage
    corrected = [sum(data_dict_corr.get(k, []))/sum(data_dict[k]) for k in labels]

    # print(f"{title} - Data")

    # for corr in corrected:
    #     print(corr)

    # Posizioni x e larghezza
    x = np.arange(len(labels))
    width = 0.6

    # Barra piena (dati originali)
    plt.bar(x, values, width=width, label='LLM response', color='skyblue')

    # Barra tratteggiata in overlay (dati corretti)
    plt.bar(x, corrected,
            width=width,
            label='Operator corrections',
            color='none',
            edgecolor='black',
            hatch='///')

    # Etichette e stile
    plt.xticks(x, labels, rotation=45, ha='right')
    plt.ylabel('Error percentage')
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.show()





if __name__ == '__main__':

    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    
    hall_dict_aut, error_plan_dict_aut, tot_hall_aut, tot_err_plan_aut, hall_dict_mitl, error_plan_dict_mitl, tot_hall_mitl, tot_err_plan_mitl, hall_corr_dict_mitl, error_plan_corr_dict_mitl, tot_hall_corr_mitl, tot_err_plan_corr_mitl = extract_metrics(json_data)


    metrics_results_trials, metrics_results_n_sens = compute_SR_and_EFF_metrics(json_data)

    # SR and Eff per n_activated_sensors:
    create_subplots_n_sens(metrics_results_n_sens)

    # # Plot 2 grafici a barre per SR ed Efficienza
    # for experiment_type, metrics in metrics_results_trials.items():
    #     plot_metrics(metrics, experiment_type)


    # HALLUCINATIONS - AUTONOMOUS
    plot_stacked_bar(hall_dict_aut, tot_hall_aut, "Source of Hallucinations - Autonomous")
    # ERROR PLAN - AUTONOMOUS
    plot_stacked_bar(error_plan_dict_aut, tot_err_plan_aut, "Source of Error Plans - Autonomous")

    # HALLUCINATIONS - MITL
    plot_stacked_bar(hall_dict_mitl, tot_hall_mitl, "Source of Hallucinations - MITL")
    # ERROR PLAN - MITL
    plot_stacked_bar(error_plan_dict_mitl, tot_err_plan_mitl, "Source of Error Plans - MITL")

    # # HALLUCINATIONS CORRECTION- MITL
    # plot_stacked_bar_correction(hall_dict_mitl, hall_corr_dict_mitl, "Source of Hallucinations + corrections - MITL")
    # # ERROR PLAN CORRECTION- MITL
    # plot_stacked_bar_correction(error_plan_dict_mitl, error_plan_corr_dict_mitl, "Source of Error Plans + corrections - MITL")


    
