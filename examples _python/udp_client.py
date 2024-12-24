import socket
import time

msgFromClient       = "Give an abgle"
bytesToSend         = str.encode(msgFromClient)
serverAddressPort   = ("192.168.1.11", 20001)
bufferSize          = 1024

# Create a UDP socket at client side

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket

while(True):
    
     UDPClientSocket.sendto(bytesToSend, serverAddressPort) 

     msgFromServer = UDPClientSocket.recvfrom(bufferSize)

     msg = "Message from Server {}".format(msgFromServer[0])

     print(msg)
     time.sleep(1)