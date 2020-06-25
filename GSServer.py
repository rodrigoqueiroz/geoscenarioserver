__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import sys
import socket
import struct
import autograd.numpy as anp
from pymoo.algorithms.nsga2 import NSGA2
from pymoo.factory import get_problem
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter
from pymoo.model.problem import Problem
from GSOptProblem import GSOptProblem
import gsc/GSParser
import msg/simulationmsg_pb2

class GSServer(Problem):

	def __init__(self):
		self._connection = None

	def load_scenario(self, file):
		GSParser = GSParser.GSParser()
		GSParser.load_and_validate_geoscenario(file)
		GSParser.report.print()
		GSParser.print_stats()
		GSParser.print_scenario()

	def start_socket(self):
		# Create a TCP/IP socket
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Bind the socket to the port
		server_address = ('localhost', 10001)
		print('starting socket on {} port {}'.format(*server_address))
		sock.bind(server_address)

		# Listen for incoming connections and wait
		sock.listen(1)
		print('waiting for simulation client')
		self._connection, client_address = sock.accept() #block
		print('connection accepted from', client_address)

	def close_connection(self):
		print('closing server socket')
		self._connection.close()

	def send_message(self, msg, type = 0):
		print('sending new data to sim client')
		sock = self._connection

		serialmsg = msg.SerializeToString()
		msglen = len(serialmsg) 
		pack1 = struct.pack('>I',msglen)
		sock.sendall(pack1+serialmsg)
		#self._connection.sendall(b'Hello Stranger')

	def read_message(self):
		print('reading data from client')
		sock = self._connection
		serialmsg = ''
		try:
			totallen = sock.recv(4) 
			if (len(totallen)==0):
				raise RuntimeError('client closed connection')
			msglen = struct.unpack('>I',totallen)[0]
			serialmsg = sock.recv(msglen)
			if (len(serialmsg)==0):
				raise RuntimeError('client closed connection')
			print('received {!r}'.format(serialmsg))
			#data = sock.recv(256) #block
	    	#if data:
		except Exception as error:
			print('Error: ' + repr(error))
		return serialmsg, 1

	def start_optimization(self):
		print('Start Opt Problem')

		#mask = ["int", "real"]
		problem = GSOptProblem(n_var=3,n_obj=1, n_constr=0, 
								xl=anp.array([0,0,0]),xu=anp.array([2,2,2]),
								simulation_function = self.run_simulation)

		#NSGA2 PARAMETERS:
		#pop_size: population size per gen
		#n_offsprings: number of offssprings per gen.
		#sampling: 
		#crossover: 
			#sbx simulated binary crossover
			#prob:mutation probability and 
		#muation:
		algorithm = NSGA2(pop_size=10, 
							n_offsprings=10, 
							sampling=get_sampling("bin_random")
							crossover=get_crossover("real_sbx", prob=0.9, eta=15), 
							mutation=get_mutation("real_pm", eta=20),
							eliminate_duplicates=True
							callback=self.callback)
		
		
		#seed: Random seed to be used. If None, a random seed is chosen. 
		#the seed is stored in the result object for reproducibility
		#ngen: number of generations
		res = minimize(problem,algorithm,('n_gen', 10),seed=1,verbose=True)
	
	def callback(algorithm, numeval, pop):
		print (x)


	def run_simulation(self,pop):
		f = []
		for indiv in pop:
			print(indiv)
			pedestrian = simulationmsg_pb2.Pedestrian()
			pedestrian.name = "p1" #todo, keep original geoscenario ob and only mix variables
			pedestrian.velocity = indiv[0] #todo: make a map with types and their array positions
			pedestrian.orientation = indiv[1]
			#pedestrian.returnpoint = x[2] #todo
			#pedestrian.startdistancetoego = x[2] #todo

			print('request simulation')
			self.send_message(pedestrian,2);
			#read results
			serialmsg, msgtype = self.read_message()
			if (msgtype==1): #results
				results = simulationmsg_pb2.ScenarioResult()
				results.ParseFromString(serialmsg)
				f.append(results.mindistance)
		#f1 = pop[:,0]**2 + pop[:,1]**2 #tests
		#f1 = f
		print('function f1')
		print(f)
		return f;



#START

if len(sys.argv) is not 2:
    print("Error: expected a single argument with a GeoScenario file.")
    exit()

#SCenario Parsing and Checking
file = sys.argv[1]
gsserver = GSServer()
gsserver.load_scenario(file)

#Simulation 
#Will block til a connection
gsserver.start_socket()

#Sampling
#TODO

#Optimization
gsserver.start_optimization()

#test msg
#for  i in range(10):
#	scenario = simulationmsg_pb2.Scenario()
#	scenario.name = "sample name"
#	serialmsg = scenario.SerializeToString()
#	print('request simulation')
#	gsserver.send_message(serialmsg);

#Server
print('end')
gsserver.close_connection()


#while True:
#	 try:


#while True:
    # Wait for a connection //block
    #connection, client_address = sock.accept()
    
 #   try:
        #Receive the data in small chunks and retransmit it
		#connection.sendall(b'Hello Stranger')
        #print('sending new data the client')
    #        data = connection.recv(16)
    #        print('received {!r}'.format(data))
    #        if data:
    #            print('sending new data the client')
                #connection.sendall(data)
    #            connection.sendall(b'Hello Stranger')
    #        else:
    #            print('no data from', client_address)
    #            break
    #finally:
        # Clean up the connection
        #connection.close()




	

