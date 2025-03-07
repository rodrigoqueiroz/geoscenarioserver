import copy

from multiprocessing import Manager, Value

from sv.VehicleBase import Vehicle

manager = Manager()

# Singleton
center_id             = Value('i', -1)
vehicles              = manager.dict()
vehicles_synchronized = Value('b', False)

def get_center_id():
	return center_id.value

def get_vehicles():	
	with vehicles_synchronized.get_lock():
		if vehicles_synchronized.value:
			return vehicles.copy()

	raise Exception('Vehicles has not yet been synchronized...')

def set_center_id(value):
	with center_id.get_lock():
		center_id.value = value

	with vehicles_synchronized.get_lock():
		vehicles_synchronized.value = False

	# Wipeout the memory since we have a new perspective
	for vid, vehicle in list(vehicles.items()):
		del vehicles[vid]


def set_vehicles(vehicles_to_sync):
	with vehicles_synchronized.get_lock():

		# Clean up to avoid trailing vehicle observations
		for vid, vehicle in list(vehicles.items()):
			del vehicles[vid]

		for vid, vehicle in list(vehicles_to_sync.items()):
			vehicle_copy = Vehicle(vid)
			vehicle_copy.type = vehicle.type
			vehicle_copy.sim_state = vehicle.sim_state
			vehicle_copy.state.set_state_vector(vehicle.state.get_state_vector())
			vehicle_copy.name = vehicle.name

			vehicles[vid] = vehicle_copy

		vehicles_synchronized.value = True