from openai import OpenAI
import json
import os



def obtainModelList(OpenAI_Client, script_dir):
    model_list = {
        "models":[]
    }

    for model in OpenAI_Client.models.list():

        model_dict = {
            "model_name":model.id,
            "owner":model.owned_by
        }
        model_list["models"].append(model_dict)

    file_path = os.path.join(script_dir, 'Open AI models.json')

    with open(file_path, 'w') as fp:
        json.dump(model_list, fp, indent=2)

    return 


def retrieveUpdatedAssistantList(OpenAI_Client, script_dir):

    names_list = []
    ids_list = []

    for assistant in OpenAI_Client.beta.assistants.list():

        names_list.append(assistant.name)
        ids_list.append(assistant.id)


    return names_list, ids_list


def extractAssistantFromJson(OpenAI_Client, script_dir):

    names_list = []
    ids_list = []

    file_path = os.path.join(script_dir, 'Open AI assistants.json')

    with open(file_path) as json_file:
        assistants_dictionary = json.load(json_file)

    for dictionary in assistants_dictionary["assistants"]:
        names_list.append(dictionary["assistant_name"])
        ids_list.append(dictionary["assistant_ID"])

    return names_list, ids_list


def updateAssistantJsonFile(names_list, ids_list, script_dir):
    assistants_dict = {
        "assistants":[]
    }

    for assistant,id in zip(names_list, ids_list):

        single_assist_dict = {
                "assistant_name":assistant,
                "assistant_ID":id
            }
        assistants_dict["assistants"].append(single_assist_dict)

    file_path = os.path.join(script_dir, 'Open AI assistants.json')

    with open(file_path, 'w') as fp:
        json.dump(assistants_dict, fp, indent=2)


def extractThreadsFromJson(OpenAI_Client, script_dir):
    threads_name_list = []
    threads_id_list = []

    file_path = os.path.join(script_dir, 'Open AI threads.json')

    with open(file_path) as json_file:
        threads_dictionary = json.load(json_file)

    for dictionary in threads_dictionary["threads"]:
        threads_name_list.append(dictionary["thread_name"])
        threads_id_list.append(dictionary["thread_ID"])

    return threads_name_list, threads_id_list

def updateThreadsJsonFile(names_list, ids_list, script_dir):
    threads_dict = {
        "threads":[]
    }

    for assistant,id in zip(names_list, ids_list):

        single_assist_dict = {
                "thread_name":assistant,
                "thread_ID":id
            }
        threads_dict["threads"].append(single_assist_dict)

    file_path = os.path.join(script_dir, 'Open AI threads.json')

    with open(file_path, 'w') as fp:
        json.dump(threads_dict, fp, indent=2)