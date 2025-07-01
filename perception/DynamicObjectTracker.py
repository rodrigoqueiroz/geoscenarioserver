import numpy as np

from math import sqrt

from dash.DashboardSharedMemory import get_center_id, set_vehicles_tracked
from SimConfig                  import PLANNER_RATE
from sv.SDVTrafficState         import fill_occupancy
from sv.VehicleBase    			import Vehicle
from util.Transformations       import (frenet_to_sim_position, OutsideRefPathException, sim_to_frenet_frame)
from util.Utils                 import distance_2p

class DynamicObjectTracker:
	def __init__(self, vehicle, detection_range, tracking_method, alpha_heat_size, alpha_min_size = 0.25, tick_required_to_drop_dynamic_object = 5):
		self.alpha_heat_size = alpha_heat_size
		self.alpha_min_size  = alpha_min_size
		self.current_heat    = {}

		self.detection_range = detection_range
		self.dynamic_objects = {
			'vehicles': {}
		}

		self.reference_position = [0., 0.]
		self.tick_required_to_drop_dynamic_object = tick_required_to_drop_dynamic_object
		self.tracking_method = tracking_method
		self.vehicle         = vehicle

		self.uids = {
			'vids': {}
		}

	def add_in_collection(self, traffic_state, key, uid, new_state):
		elements = getattr(traffic_state, key)
		elements[uid] = new_state
		setattr(traffic_state, key, elements)

	def alpha_beta(self, traffic_state, sdv_route, collection, collection_key, euclid_key, frenet_key, tick_state, elapsed_time):
		def euclid_alpha_beta():
			predicted_vector = collection[collection_key].future_euclidian_state(elapsed_time)

			[ obs_x, obs_x_vel, obs_x_acc ] = tick_state.state.get_X()
			[ obs_y, obs_y_vel, obs_y_acc ] = tick_state.state.get_Y()

			[ pred_x, pred_x_vel, pred_x_acc ] = predicted_vector[:3]
			[ pred_y, pred_y_vel, pred_y_acc ] = predicted_vector[3:]

			distance   = distance_2p(self.vehicle.state.x, self.vehicle.state.y, obs_x, obs_y)
			alpha_size = min(distance / self.detection_range, 1.0)

			collection[collection_key].state.x     = step(alpha_size, pred_x,     obs_x)
			collection[collection_key].state.x_vel = step(alpha_size, pred_x_vel, obs_x_vel)
			collection[collection_key].state.x_acc = step(alpha_size, pred_x_acc, obs_x_acc)
			collection[collection_key].state.y     = step(alpha_size, pred_y,     obs_y)
			collection[collection_key].state.y_vel = step(alpha_size, pred_y_vel, obs_y_vel)
			collection[collection_key].state.y_acc = step(alpha_size, pred_y_acc, obs_y_acc)

			try:
				s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_reference_path(), collection[collection_key].state.get_X(), 
														 collection[collection_key].state.get_Y(), sdv_route.get_reference_path_s_start())

				collection[collection_key].state.set_S(s_vector)
				collection[collection_key].state.set_D(d_vector)
				self.add_in_collection(traffic_state, frenet_key, collection_key, collection[collection_key])

			except OutsideRefPathException:
				collection[collection_key].state.set_S([0.0, 0.0, 0.0])
				collection[collection_key].state.set_D([0.0, 0.0, 0.0])
				self.add_in_collection(traffic_state, euclid_key, collection_key, collection[collection_key])

			return alpha_size

		def frenet_alpha_beta(observed_vector):
			alpha_size    = 0.0
			is_successful = False

			if np.any(observed_vector):
				predicted_vector = collection[collection_key].future_state(elapsed_time)

				reference_x, reference_y = self.reference_position
				s_shift, d_shift = sim_to_frenet_frame(sdv_route.get_reference_path(), [reference_x, 0, 0], 
													   [reference_y, 0, 0], sdv_route.get_reference_path_s_start())

				[ obs_s, obs_s_vel, obs_s_acc ]    = observed_vector[:3]
				[ obs_d, obs_d_vel, obs_d_acc ]    = observed_vector[3:]

				[ pred_s, pred_s_vel, pred_s_acc ] = predicted_vector[:3]
				[ pred_d, pred_d_vel, pred_d_acc ] = predicted_vector[3:]

				pred_s += s_shift[0]
				pred_d += d_shift[0]

				distance   = distance_2p(self.vehicle.state.s, self.vehicle.state.d, obs_s, obs_d)
				alpha_size = min(distance / self.detection_range, 1.0)
				
				collection[collection_key].state.s     = step(alpha_size, pred_s,     obs_s)
				collection[collection_key].state.s_vel = step(alpha_size, pred_s_vel, obs_s_vel)
				collection[collection_key].state.s_acc = step(alpha_size, pred_s_acc, obs_s_acc)
				collection[collection_key].state.d     = step(alpha_size, pred_d,     obs_d)
				collection[collection_key].state.d_vel = step(alpha_size, pred_d_vel, obs_d_vel)
				collection[collection_key].state.d_acc = step(alpha_size, pred_d_acc, obs_d_acc)

				try:
					x, y = frenet_to_sim_position(sdv_route.get_reference_path(), collection[collection_key].state.s, 
						                          collection[collection_key].state.d, sdv_route.get_reference_path_s_start())
					
					x_vel = (x - collection[collection_key].state.x) / elapsed_time
					y_vel = (y - collection[collection_key].state.y) / elapsed_time
					x_acc = (x_vel - collection[collection_key].state.x_vel) / elapsed_time
					y_acc = (y_vel - collection[collection_key].state.y_vel) / elapsed_time

					collection[collection_key].state.x     = x
					collection[collection_key].state.y     = y
					collection[collection_key].state.x_vel = x_vel
					collection[collection_key].state.y_vel = y_vel
					collection[collection_key].state.x_acc = x_acc
					collection[collection_key].state.y_acc = y_acc

					self.add_in_collection(traffic_state, frenet_key, collection_key, collection[collection_key])
					is_successful = True

				except OutsideRefPathException:
					pass # Ignore

			return alpha_size, is_successful


		def step(alpha_size, prediction, observed_value):
			'''if self.current_heat[tick_state.id] < self.alpha_heat_size:
				alpha = self.current_heat[tick_state.id] / self.alpha_heat_size * alpha_size
			else:
				alpha = self.alpha_min_size + (1.0 - self.alpha_min_size) * alpha_size'''

			alpha = min(self.current_heat[tick_state.id], self.alpha_heat_size) / self.alpha_heat_size * alpha_size

			alpha_mean =        alpha  * prediction
			beta__mean = (1.0 - alpha) * observed_value

			return alpha_mean + beta__mean
		
		# Incompatible Tracking Method, die silently
		if collection_key not in collection or tick_state == None:
			return

		if tick_state.id not in self.current_heat:
			self.current_heat[tick_state.id] = 0

		self.current_heat[tick_state.id] += 1
		observed_vector = np.array(tick_state.state.get_S() + tick_state.state.get_D())
		#alpha_size, is_successful = frenet_alpha_beta(observed_vector)

		#if not is_successful:
		alpha_size = euclid_alpha_beta()
			
		# Regardless of the alpha_beta method
		obs_yaw  = tick_state.state.yaw
		pred_yaw = collection[collection_key].state.yaw
		collection[collection_key].state.yaw   = step(alpha_size, pred_yaw, obs_yaw)


	def memorize_dynamic_objects(self, traffic_state, sdv_route, dynamic_object_key, euclid_key, frenet_key, uid_key, elapsed_time):
		all_objects = {}
		all_objects.update(getattr(traffic_state, euclid_key))
		all_objects.update(getattr(traffic_state, frenet_key))

		setattr(traffic_state, euclid_key, {})
		setattr(traffic_state, frenet_key, {})

		for uid, dynamic_object in list(all_objects.items()):
			self.uids[uid_key][uid] = self.tick_required_to_drop_dynamic_object
			self.update_dynamic_object(traffic_state, sdv_route, self.dynamic_objects[dynamic_object_key], 
				                       uid, euclid_key, frenet_key, dynamic_object, elapsed_time)

		for uid in list(self.uids[uid_key]):
			self.uids[uid_key][uid] -= 1

			# Drop A Dynamic Object from the Memory
			if self.uids[uid_key][uid] < 0:
				del self.dynamic_objects[dynamic_object_key][uid]
				del self.uids[uid_key][uid]

				if uid in self.current_heat:
					del self.current_heat[uid]

			elif self.uids[uid_key][uid] < self.tick_required_to_drop_dynamic_object - 1:
				self.update_dynamic_object(traffic_state, sdv_route, self.dynamic_objects[dynamic_object_key], 
					                       uid, euclid_key, frenet_key, None, elapsed_time)


	def track_dynamic_objects(self, traffic_state, lanelet_map, sdv_route, elapsed_time):
		"""
			Mutate the traffic state with respect to dynamic objects and road occupancy
		"""
		elapsed_time = 0.1

		if self.tracking_method == None or not hasattr(self, self.tracking_method):
			return traffic_state

		self.memorize_dynamic_objects(traffic_state, sdv_route, 'vehicles', 'traffic_vehicles_orp', 'traffic_vehicles', 'vids', elapsed_time)
		traffic_state.road_occupancy = fill_occupancy(self.vehicle, traffic_state.lane_config, traffic_state.traffic_vehicles, 
													  traffic_state.traffic_vehicles_orp, lanelet_map, traffic_state.intersections)

		# For data visualization
		if get_center_id() == traffic_state.vid:
			vehicles = {}
			vehicles.update(traffic_state.traffic_vehicles)
			vehicles.update(traffic_state.traffic_vehicles_orp)

			set_vehicles_tracked(vehicles)
			self.vehicles = vehicles

		self.reference_position = frenet_to_sim_position(sdv_route.get_reference_path(), 0.0, 0.0, sdv_route.get_reference_path_s_start())

		# Has mutated
		return traffic_state

	def update_dynamic_object(self, traffic_state, sdv_route, collection, collection_key, euclid_key, frenet_key, tick_state, elapsed_time):
		def euclid_update():
			euclid_state = collection[collection_key].future_euclidian_state(elapsed_time)
			collection[collection_key].state.set_X(euclid_state[:3])
			collection[collection_key].state.set_Y(euclid_state[3:])

			try:
				s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_reference_path(), euclid_state[:3], 
														 euclid_state[3:], sdv_route.get_reference_path_s_start())

				collection[collection_key].state.set_S(s_vector)
				collection[collection_key].state.set_D(d_vector)
				self.add_in_collection(traffic_state, frenet_key, collection_key, collection[collection_key])

			except OutsideRefPathException:
				collection[collection_key].state.set_S([0.0, 0.0, 0.0])
				collection[collection_key].state.set_D([0.0, 0.0, 0.0])
				self.add_in_collection(traffic_state, euclid_key, collection_key, collection[collection_key])

		def frenet_update(frenet_vector):
			is_successful = False

			if np.any(frenet_vector):
				frenet_state = collection[collection_key].future_state(elapsed_time)

				reference_x, reference_y = self.reference_position
				s_shift, d_shift = sim_to_frenet_frame(sdv_route.get_reference_path(), [reference_x, 0, 0], 
													   [reference_y, 0, 0], sdv_route.get_reference_path_s_start())

				frenet_state[0] += s_shift[0]
				frenet_state[3] += d_shift[0]

				collection[collection_key].state.set_S(frenet_state[:3])
				collection[collection_key].state.set_D(frenet_state[3:])
				s = frenet_state[0]
				d = frenet_state[3]

				try:
					x, y = frenet_to_sim_position(sdv_route.get_reference_path(), s, d, sdv_route.get_reference_path_s_start())
					collection[collection_key].state.x = x
					collection[collection_key].state.y = y
					collection[collection_key].state.x_acc = 0.0
					collection[collection_key].state.y_axx = 0.0
					self.add_in_collection(traffic_state, frenet_key, collection_key, collection[collection_key])
					is_successful = True

				except OutsideRefPathException:
					pass # Ignore

			return is_successful


		# Dynamic object has been previously observed
		if collection_key in collection:

			# Dynamic object has not been observed this tick
			if tick_state == None:
				frenet_vector = np.array(collection[collection_key].state.get_S() +
					                     collection[collection_key].state.get_D())

				print('Vehicle ' + str(collection_key) + ' invisible...')
				#if not frenet_update(frenet_vector):
				euclid_update()

			# Dynamic object is observed and there is a motion history
			elif hasattr(self, self.tracking_method):
				getattr(self, self.tracking_method)(traffic_state, sdv_route, collection, collection_key, euclid_key, frenet_key, tick_state, elapsed_time)

		# First time we observed that dynamic object
		elif tick_state != None:
			collection[collection_key] = tick_state
			frenet_vector = np.array(tick_state.state.get_S() + tick_state.state.get_D())

			if np.any(frenet_vector):
				self.add_in_collection(traffic_state, frenet_key, collection_key, tick_state)
			else:
				self.add_in_collection(traffic_state, euclid_key, collection_key, tick_state)