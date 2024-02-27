import websockets
import asyncio

# Server data
PORT = 7890
print("Server listening on Port " + str(PORT))

# A set of connected ws clients
connected = set()
connected_clients = []

# The main behavior function for this server
async def echo(websocket, path):
    print("A client just connected")
    # Store a copy of the connected client
    connected.add(websocket)
    connected_clients.append(websocket)
    # Handle incoming messages
    try:
        async for message in websocket:
            #print("Received message from client: " + message)
            # Send a response to all connected clients except sender
            #for conn in websocket:
                #if conn != websocket:
            print("Connected clients: ", connected_clients)
            for client in connected_clients:
                await client.send(message)
                print("Sent message: " + message)
    # Handle disconnecting clients 
    except websockets.exceptions.ConnectionClosed as e:
        print("A client just disconnected")
    finally:
        connected.remove(websocket)

# Start the server
start_server = websockets.serve(echo, "0.0.0.0", PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()