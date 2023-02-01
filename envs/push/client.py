import socket
import json

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 7000  # The port used by the server

def recvall(sock):
# Helper function to recv n bytes or return None if EOF is hit
	data = bytearray()
	while data[-1:] != b';':
		packet = sock.recv(1024)
		if packet:
			data.extend(packet)
		
	return data[:-1]

class SimulationClient:
    def __init__(self):
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      
    def connect(self, host, port):
      self.sock.connect((host, port))
       
    def request(self, req):
      data = json.dumps(req) + ';' ;
      self.sock.sendall(bytes(data,encoding="utf-8"))

    def response(self):
    # Receive data from the server and shut down
      received = recvall(self.sock)
      received = received.decode("utf-8")
      self.sock.close()
      return received ;
     
sock = SimulationClient() ;
sock.connect(HOST, PORT)
sock.request({"request": "step"})
response = sock.response()

print(f"Received {response}")
