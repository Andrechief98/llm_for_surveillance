from openai import OpenAI
import json


client = OpenAI()

from openai import OpenAI
client = OpenAI()


delete_all = True

if delete_all:

    assistantIDs_to_delete_list = []

    with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI assistants.json", "r") as file:
        assistants_dict = json.load(file)


    for assistant_dict in assistants_dict["assistants"]:
        assistantIDs_to_delete_list.append(assistant_dict["assistant_ID"])
        client.beta.assistants.delete(assistant_dict["assistant_ID"])

    empty_dict = {
        "assistants" : []
    }

    with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI assistants.json", "w") as file:
        json.dump(empty_dict, file, indent=2)

    print(f"Deleted assistants: \n {assistantIDs_to_delete_list}")
    

else:
    # TO DO: update the file deleting only the sequence of deleted assistants
    assistants_to_delete = [
    "asst_h0bA50bwsaGnwDsaZO6FuIYK"
    ]

    for assistant in assistants_to_delete:
        response = client.beta.assistants.delete(assistant)
        print(response)











