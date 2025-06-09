from openai import OpenAI
import json


client = OpenAI()

from openai import OpenAI
client = OpenAI()

delete_all = True

if delete_all:

    threadsIDs_to_delete_list = []

    with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI threads.json", "r") as file:
        threads_dict = json.load(file)


    for thread_dict in threads_dict["threads"]:
        threadsIDs_to_delete_list.append(thread_dict["thread_ID"])
        client.beta.threads.delete(thread_dict["thread_ID"])

    empty_dict = {
        "threads" : []
    }

    with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI threads.json", "w") as file:
        json.dump(empty_dict, file, indent=2)

    print(f"Deleted threads: \n {threadsIDs_to_delete_list}")


else:
    # TO DO: update the file deleting only the sequence of deleted threads
    threads_to_delete = [
        "thread_F4RBh4exGjNJ37BIv8b0GB3d"
        ]

    for thread in threads_to_delete:
        response = client.beta.threads.delete(thread)
        print(response)



