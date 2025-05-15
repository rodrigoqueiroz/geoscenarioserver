import copy

from multiprocessing import Manager, Value

from sv.VehicleBase import Vehicle

manager = Manager()

# Singleton
center_id                       = Value('i', -1)
vehicles_perceived              = manager.dict()
vehicles_perceived_synchronized = Value('b', False)
vehicles_tracked                = manager.dict()
vehicles_tracked_synchronized   = Value('b', False)

def get_center_id():
	return center_id.value

def get_vehicles():
	# Favor vehicles from the Tracker when available
	with vehicles_tracked_synchronized.get_lock():
		if vehicles_tracked_synchronized.value:
			return vehicles_tracked.copy()

	# Fallback to vehicles from Perception
	with vehicles_perceived_synchronized.get_lock():
		if vehicles_perceived_synchronized.value:
			return vehicles_perceived.copy()

	# Fallback to ground truth
	raise Exception('Vehicles has not yet been synchronized...')

def override_vehicles(collection, vehicles_to_sync):
	wipeout(collection)

	for vid, vehicle in list(vehicles_to_sync.items()):
		vehicle_copy = Vehicle(vid)
		vehicle_copy.type = vehicle.type
		vehicle_copy.sim_state = vehicle.sim_state
		vehicle_copy.state.set_state_vector(vehicle.state.get_state_vector())
		vehicle_copy.name = vehicle.name

		collection[vid] = vehicle_copy

def set_center_id(value):
	with center_id.get_lock():
		center_id.value = value

	with vehicles_perceived_synchronized.get_lock():
		vehicles_perceived_synchronized.value = False

	wipeout(vehicles_perceived)

	with vehicles_tracked_synchronized.get_lock():
		vehicles_tracked_synchronized.value   = False

	wipeout(vehicles_tracked)


def set_vehicles_perceived(vehicles_to_sync):
	with vehicles_perceived_synchronized.get_lock():
		override_vehicles(vehicles_perceived, vehicles_to_sync)
		vehicles_perceived_synchronized.value = True

def set_vehicles_tracked(vehicles_to_sync):
	with vehicles_tracked_synchronized.get_lock():
		override_vehicles(vehicles_tracked, vehicles_to_sync)
		vehicles_tracked_synchronized.value = True

# Wipeout the memory to a new perspective
def wipeout(collection):
	for uid, _ in list(collection.items()):
		del collection[uid]