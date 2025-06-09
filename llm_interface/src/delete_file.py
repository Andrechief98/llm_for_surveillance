from openai import OpenAI


client = OpenAI()

from openai import OpenAI
client = OpenAI()


file_to_delete = [
    "file-KHPZunJChs6kC8vtPmw8AA"
    ]

for file in file_to_delete:
    response = client.files.delete(file)
    print(response)



