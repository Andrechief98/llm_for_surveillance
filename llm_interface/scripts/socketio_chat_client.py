import socketio

# Standard Python SocketIO client
sio = socketio.Client()

@sio.event
def connect():
    print('Connected to server')
    # Send a chat message after connecting
    sio.emit('chat_message', {'user_input': 'Hello, robot!'})

@sio.event
def disconnect():
    print('Disconnected from server')

@sio.on('chat_response')
def on_chat_response(data):
    print('Received response:', data['response'])

if __name__ == '__main__':
    sio.connect('http://localhost:5000')
    sio.wait()
