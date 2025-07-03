from openai import OpenAI
import json
import os

script_dir = os.path.dirname(__file__)

client = OpenAI()


# delete_all = True

# if delete_all:
#     fileIDs_to_delete_list = []

#     with open(f"{script_dir}/Open AI files.json", "r") as file:
#         files_dict = json.load(file)


#     for file_dict in files_dict["files"]:
#         fileIDs_to_delete_list.append(file_dict["file_ID"])
#         client.files.delete(file_dict["file_ID"])

#     empty_dict = {
#         "files" : []
#     }

#     with open(f"{script_dir}/Open AI files.json", "w") as file:
#         json.dump(empty_dict, file, indent=2)

#     print(f"Deleted files: \n {fileIDs_to_delete_list}")

# else:
#     file_to_delete = [
#         "file-KHPZunJChs6kC8vtPmw8AA"
#         ]

#     for file in file_to_delete:
#         response = client.files.delete(file)
#         print(response)



file_list = client.files.list()

for file in file_list:
    print(file.filename)
    print(file.id)


 
