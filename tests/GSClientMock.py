import socket
import sys
import struct
import simulationmsg_pb2




# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 10001)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)


while True:
    try:
        #recv() blocks till gets a message. Emmpty message means connection was closed
        totallen = sock.recv(4) 
        if (len(totallen)==0):
            raise RuntimeError('Server closed connection ')
        msglen = struct.unpack('>I',totallen)[0]
        serialmsg = sock.recv(msglen)
        if (len(serialmsg)==0):
            raise RuntimeError('Server closed connection ')
        print('received {!r}'.format(serialmsg))
        
        #parser
        #scenario = simulationmsg_pb2.Scenario()
        #scenario.ParseFromString(serialmsg)
        #print('Scenario name:')
        #print(scenario.name)
        pedestrian = simulationmsg_pb2.Pedestrian()
        pedestrian.ParseFromString(serialmsg)
        print(pedestrian.name)

        #reply
        result = simulationmsg_pb2.ScenarioResult()
        result.minttc = 1;
        result.mindistance = 1;
        serialmsg = result.SerializeToString()
        msglen = len(serialmsg) 
        pack1 = struct.pack('>I',msglen)
        sock.sendall(pack1+serialmsg)

    except Exception as error:
        print('Error: ' + repr(error))
        break;

print('closing client socket')
sock.close()