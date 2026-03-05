import chainlit as cl
from openai import AsyncOpenAI
import rospy
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
import actionlib
import yaml

# Inizializziamo il client asincrono
client = AsyncOpenAI()

@cl.on_chat_start
async def start():
    """Eseguito quando l'utente apre la chat"""
    # Inizializziamo la cronologia nella sessione dell'utente
    cl.user_session.set("history", [{"role": "system", "content": "Sei un assistente utile."}])
    await cl.Message(content="Ciao! Sono pronto. Cosa vuoi chiedermi?").send()

@cl.on_message
async def main(message: cl.Message):
    """Eseguito ogni volta che l'utente invia un messaggio"""
    
    # Recuperiamo la cronologia
    history = cl.user_session.get("history")
    history.append({"role": "user", "content": message.content})

    # Creiamo un messaggio vuoto che "riempiremo" con lo streaming
    msg = cl.Message(content="")
    
    # Chiamata all'LLM in modalità streaming
    stream = await client.chat.completions.create(
        model="gpt-4",
        messages=history,
        stream=True
    )

    async for part in stream:
        if token := part.choices[0].delta.content:
            # Inviamo il token all'interfaccia in tempo reale
            await msg.stream_token(token)

    # Una volta finito lo streaming, salviamo la risposta nella cronologia
    history.append({"role": "assistant", "content": msg.content})
    cl.user_session.set("history", history)
    
    # Confermiamo l'invio finale
    await msg.send()

