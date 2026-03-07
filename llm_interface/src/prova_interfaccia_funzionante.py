import chainlit as cl
from openai import AsyncOpenAI
import rospy
import json




class RobotAssistant:
    def __init__(self):
        self.client = AsyncOpenAI()

        # Inizializziamo la cronologia nella sessione se non esiste
        if not cl.user_session.get("history"):
            cl.user_session.set("history", [])
    
    async def handle_message(self, user_input):

        # Riprendiamo lo storico della conversazione a cui aggiungeremo il nuovo messaggio
        history = cl.user_session.get("history")
        history.append({"role": "user", "content": user_input})

        # Creiamo un messaggio vuoto che "riempiremo" con lo streaming
        msg = cl.Message(content="")
        
        stream = await self.client.responses.create(
            model="gpt-4", # O gpt-4o
            input=history,
            tools = [
                {
                    "type": "function",  # Specifica sempre il tipo
                    "name": "send_robots_to_area",  # <--- Il parametro mancante era qui
                    "description": "Invia una sequenza di robot in un'area specifica",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "robots_sequence": {
                                "type": "string",
                                "description": "Lista dei robot e destinazioni"
                            }
                        },
                        "required": ["robots_sequence"]
                    }
                },
                {
                    "type": "function",  # Specifica sempre il tipo
                    "name": "retrieve_system_state",  # <--- Il parametro mancante era qui
                    "description": "Fornisce lo stato del sistema",
                    "parameters": {}
                },
            ],
            stream=True
        )


        full_ai_response = ""
        final_tool_calls = {}
        active_steps = {} # Teniamo traccia degli step attivi per indice
        tool_outputs = []

        async for event in stream:
            # 1. GESTIONE TESTO
            if event.type == "response.created":
                pass # Ottimo per loggare l'ID della risposta
            
            elif event.type == "response.failed":
                # Per avvisare direttamente in chat che c'è stato un errore
                await cl.Message(content=f"Error: {event.error.message}").send()
                break

            # --- CASI DEL TESTO (OUTPUT) ---
            elif event.type == "response.output_text.delta":
                if not msg.id: 
                    await msg.send()
                full_ai_response += event.delta
                await msg.stream_token(event.delta)

            elif event.type == "response.text.done":
                # Testo terminato, possiamo aggiornare lo stato finale
                await msg.update()

            # CASI DEI TOOL (FUNZIONI ROS)
            elif event.type == "response.output_item.added":
                # Viene aggiunto un nuovo item (testo o funzione)
                if event.item.type == "function_call":
                    idx = event.output_index
                    tool_name = event.item.name 
                    
                    # Creiamo uno step professionale
                    step = cl.Step(name=tool_name, type="tool")
                    step.language = "json" # Se vuoi mostrare i parametri
                    await step.send()

                    final_tool_calls[idx] = event.item # passiamo l'intero item 
                    active_steps[idx] = step

            elif event.type == "response.function_call_arguments.delta":
                idx = event.output_index
                if idx in final_tool_calls:
                    # Accumulo asincrono dei parametri JSON
                    if final_tool_calls[idx].arguments is None:
                        final_tool_calls[idx].arguments = ""
                    final_tool_calls[idx].arguments += event.delta

                if idx in active_steps:
                    if active_steps[idx].input is None:
                        active_steps[idx].input = ""
                    
                    await active_steps[idx].stream_token(event.delta, is_input=True)

            elif event.type == "response.function_call_arguments.done":
                # Parametri pronti!
                idx = event.output_index
                tool_call = final_tool_calls[idx]
                step = active_steps[idx] # Recuperiamo lo step creato in .added
                args = event.arguments # event.arguments è la stringa completa
                
                # Esecuzione ROS effettiva
                result = await cl.make_async(self.execute_ros_command)(tool_call.name, json.loads(args))

                # Chiude lo step legato all'output
                active_steps[idx].output = result
                await active_steps[idx].update()


                # Aggiungi la chiamata "completata"
                history.append({
                    "type": "function_call",
                    "call_id": tool_call.call_id,
                    "name": tool_name,
                    "arguments": args
                })

                # Aggiungi l'output di ROS
                history.append({
                    "type": "function_call_output",
                    "call_id": tool_call.call_id,
                    "output": str(result)
                })


                # Aggiunta ai tool chiamati in modo che sia possibile verificare la presenza di tool
                tool_outputs.append({
                    "type": "function_call_output",
                    "call_id": tool_call.call_id,
                    "output": json.dumps({
                            "system_state": result
                        })
                })

                

            # CASI DI RAGIONAMENTO (REASONING)
            elif event.type == "response.reasoning.delta":
                # Se usi modelli o1/o3, qui arriva il "pensiero" del modello.
                # Puoi scegliere se mostrarlo o ignorarlo.
                pass

            elif event.type == "response.completed":
                if msg.id:
                    await msg.update()
                break # Esci dal ciclo per sicurezza
        
        if full_ai_response:
            history.append({"role": "assistant", "content": full_ai_response})
        
        if tool_outputs:
                
            follow_up_stream = await self.client.responses.create(
                model="gpt-4o",
                input=history, # La storia ora include l'esito dell'azione ROS
                tools = [],
                stream=True
            )
    
            # Continuiamo lo streaming nel messaggio originale 'msg'
            tool_function_result_msg = ""
            async for event in follow_up_stream:
                if event.type == "response.created":
                    pass # Ottimo per loggare l'ID della risposta
                
                elif event.type == "response.failed":
                    # Per avvisare direttamente in chat che c'è stato un errore
                    await cl.Message(content=f"Error: {event.error.message}").send()
                    break

                elif event.type == "response.output_text.delta":
                    if not msg.id: 
                        await msg.send()
                    tool_function_result_msg += event.delta
                    await msg.stream_token(event.delta)

                elif event.type == "response.completed":
                    if msg.id:
                        await msg.update()
                    break # Esci dal ciclo per sicurezza
            
            if tool_function_result_msg:
                history.append({"role": "assistant", "content": full_ai_response})
        
        cl.user_session.set("history", history)


    def execute_ros_command(self, tool_name, args):
        """
        Dispatcher che smista le chiamate ai metodi ROS specifici.
        Viene chiamata tramite cl.make_async per non bloccare la UI.
        """
        try:
            # 1. Verifica se il metodo esiste nella classe
            if hasattr(self, tool_name):
                method = getattr(self, tool_name)
                
                # 2. Log per il terminale ROS
                rospy.loginfo(f"Esecuzione tool: {tool_name} con argomenti: {args}")
                
                # 3. Chiamata dinamica al metodo (es. self.send_robots_to_area(**args))
                # L'operatore ** scompatta il dizionario args nei parametri della funzione
                result = method(**args)
                
                return result
            else:
                error_msg = f"Errore: Il comando '{tool_name}' non è implementato nel nodo ROS."
                rospy.logerr(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Eccezione durante l'esecuzione di {tool_name}: {str(e)}"
            rospy.logerr(error_msg)
            return error_msg
    
    def send_robots_to_area(self, robots_sequence):
        """Esempio di comando di movimento"""
        # Qui inserisci la tua logica ROS Noetic (es. pubblicazione su un topic o chiamata service)
        # Esempio: self.my_publisher.publish(msg)
        return f"Sequenza '{robots_sequence}' inviata correttamente ai robot."

    def retrieve_system_state(self):
        """Esempio di lettura dati dai sensori"""
        # Qui potresti leggere una variabile aggiornata da un Subscriber
        return "Stato sistema: Batteria 90%, Tutti i sensori attivi."


# --- Integrazione Chainlit ---
@cl.on_chat_start
async def start():
    agent = RobotAssistant()
    cl.user_session.set("agent", agent)
    

@cl.on_message
async def main(message: cl.Message):
    """Eseguito ogni volta che l'utente invia un messaggio"""
    
    agent = cl.user_session.get("agent")
    await agent.handle_message(message.content)