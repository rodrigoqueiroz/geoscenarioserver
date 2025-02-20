import numpy as np

from math import sqrt

from dash.DashboardSharedMemory import get_center_id, set_vehicles
from sv.SDVTrafficState         import fill_occupancy
from sv.VehicleBase    			import Vehicle
from util.Transformations       import (frenet_to_sim_position, OutsideRefPathException, sim_to_frenet_frame)

class Perception:
	def __init__(self, vid, detection_range_in_meters, hallucination_retention, hallucination_weight, missed_detection_weight, 
				 noise_position_mixture, noise_yaw_mostly_reliable, noise_yaw_strongly_inaccurate, seed=1):
		self.all_vehicles              = {}
		self.detection_range_in_meters = detection_range_in_meters
		self.hallucination_index       = -1
		self.hallucination_retention   = hallucination_retention
		self.hallucination_status      = {}
		self.hallucination_weight      = hallucination_weight
		self.missed_detection_weight   = missed_detection_weight
		self.my_vid                    = vid
		self.position_mean             = noise_position_mixture[0]
		self.position_std              = noise_position_mixture[1]
		self.random                    = np.random
		self.rng_tick                  = 0
		self.vehicles                  = {}
		self.seed                      = seed
		self.yaw_mostly_reliable       = noise_yaw_mostly_reliable
		self.yaw_strongly_inaccurate   = noise_yaw_strongly_inaccurate

	def apply_noise(self, traffic_state, lanelet_map, sdv_route):
		# For reproducible pseudo generation of random numbers
		self.rng_tick += 1
		self.random.seed(self.my_vid + self.seed + self.rng_tick)

		if self.detection_range_in_meters == None:
			return traffic_state

		self.all_vehicles = {}
		self.all_vehicles.update(traffic_state.traffic_vehicles)
		self.all_vehicles.update(traffic_state.traffic_vehicles_orp)
		ego_vehicle_state = traffic_state.vehicle_state
		traffic_state.traffic_vehicles     = {}
		traffic_state.traffic_vehicles_orp = {}

		self.update_hallucination(ego_vehicle_state, traffic_state, sdv_route)
		#self.hallucinate_its_own_reflection(ego_vehicle_state, traffic_state)

		for vid, vehicle in list(self.all_vehicles.items()):
			if self.is_detecting_that_vehicle(ego_vehicle_state, vehicle):
				vehicle = self.apply_yaw_noise(ego_vehicle_state, vehicle)
				vehicle, has_frenet_location = self.apply_positional_noise(ego_vehicle_state, vid, vehicle, sdv_route)

				#self.hallucinate_splitting_bounding_box(ego_vehicle_state, vid, vehicle, traffic_state, sdv_route)

				if has_frenet_location:
					traffic_state.traffic_vehicles[vid]     = vehicle
				else:
					traffic_state.traffic_vehicles_orp[vid] = vehicle

		traffic_state.road_occupancy = fill_occupancy(self.my_vid, traffic_state.vehicle_state, traffic_state.lane_config, 
													  traffic_state.traffic_vehicles, traffic_state.traffic_vehicles_orp, 
                                                      lanelet_map, traffic_state.intersections)

		# For data visualization
		if get_center_id() == traffic_state.vid:
			vehicles = {}
			vehicles.update(traffic_state.traffic_vehicles)
			vehicles.update(traffic_state.traffic_vehicles_orp)

			set_vehicles(vehicles)
			self.vehicles = vehicles

		# Has mutated
		return traffic_state

	def apply_gaussian_noise(self, intensity, ground_truth, mixture):
		[mean, std_dev] = mixture
		gaussian_noise  = self.random.normal(mean, std_dev)

		return ground_truth + intensity * gaussian_noise

	def apply_positional_noise(self, ego_vehicle_state, vid, other_vehicle, sdv_route, tracked_hallucination=False):
		distance  = self.distance(ego_vehicle_state, other_vehicle)
		intensity = distance / self.detection_range_in_meters
		mixture   = [ self.position_mean, self.position_std ]
		has_frenet_location = True

		other_vehicle.state.x = self.apply_gaussian_noise(intensity, other_vehicle.state.x, mixture)
		other_vehicle.state.y = self.apply_gaussian_noise(intensity, other_vehicle.state.y, mixture)
		other_vehicle.state.z = self.apply_gaussian_noise(intensity, other_vehicle.state.z, mixture)

		if vid in self.vehicles:
			other_vehicle.state.x_vel = other_vehicle.state.x     - self.vehicles[vid].state.x
			other_vehicle.state.y_vel = other_vehicle.state.y     - self.vehicles[vid].state.y
			other_vehicle.state.z_vel = other_vehicle.state.z     - self.vehicles[vid].state.z
			other_vehicle.state.x_acc = other_vehicle.state.x_vel - self.vehicles[vid].state.x_vel
			other_vehicle.state.y_acc = other_vehicle.state.y_vel - self.vehicles[vid].state.y_vel
			other_vehicle.state.z_acc = other_vehicle.state.z_vel - self.vehicles[vid].state.z_vel
		elif tracked_hallucination:
			other_vehicle.state.x_vel = other_vehicle.state.x     - self.hallucination_status[vid]['vehicle'].state.x
			other_vehicle.state.y_vel = other_vehicle.state.y     - self.hallucination_status[vid]['vehicle'].state.y
			other_vehicle.state.z_vel = other_vehicle.state.z     - self.hallucination_status[vid]['vehicle'].state.z
			other_vehicle.state.x_acc = other_vehicle.state.x_vel - self.hallucination_status[vid]['vehicle'].state.x_vel
			other_vehicle.state.y_acc = other_vehicle.state.y_vel - self.hallucination_status[vid]['vehicle'].state.y_vel
			other_vehicle.state.z_acc = other_vehicle.state.z_vel - self.hallucination_status[vid]['vehicle'].state.z_vel
		else:
			other_vehicle.state.x_vel = 0
			other_vehicle.state.y_vel = 0
			other_vehicle.state.z_vel = 0
			other_vehicle.state.x_acc = 0
			other_vehicle.state.y_acc = 0
			other_vehicle.state.z_acc = 0

			other_vehicle.state.s_vel = 0
			other_vehicle.state.d_vel = 0
			other_vehicle.state.s_acc = 0
			other_vehicle.state.d_acc = 0

		try:
			s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_global_path(), other_vehicle.state.get_X(), 
													 other_vehicle.state.get_Y(), 0)
			other_vehicle.state.set_S(s_vector)
			other_vehicle.state.set_D(d_vector)
		except OutsideRefPathException:
			other_vehicle.state.set_S([0.0, 0.0, 0.0])
			other_vehicle.state.set_D([0.0, 0.0, 0.0])
			has_frenet_location = False

		return other_vehicle, has_frenet_location

	def apply_yaw_noise(self, ego_vehicle_state, other_vehicle, intensities=[10, 45, 90]):
		if len(intensities) == 3:
			s_vel = other_vehicle.state.s_vel if other_vehicle.state.s_vel != None else 0.0
			detection_value = self.random.random() * 2

			if detection_value > 1:
				detection_value = -detection_value / 2

			if s_vel > self.yaw_mostly_reliable:
				intensity = intensities[0]
			elif s_vel > self.yaw_strongly_inaccurate:
				intensity = intensities[1]
			else:
				intensity = intensities[2]

			yaw = other_vehicle.state.yaw + detection_value * intensity

			if yaw < -180.0:
				yaw += 360.0
			elif yaw >= 180.0:
				yaw -= 360.0

			other_vehicle.state.yaw = yaw

		return other_vehicle

	def distance(self, ego_vehicle_state, other_vehicle):
		return sqrt(
			pow(ego_vehicle_state.x - other_vehicle.state.x, 2) +
			pow(ego_vehicle_state.y - other_vehicle.state.y, 2) +
			pow(ego_vehicle_state.z - other_vehicle.state.z, 2)
		)

	def hallucinate_its_own_reflection(self, ego_vehicle_state, traffic_state):
		detection_value = self.random.random()
		likelihood      = sqrt(100.0 * self.hallucination_weight) / 100.0
		retention       = 0.8

		if detection_value < likelihood:
			bias            = self.random.random() * self.detection_range_in_meters
			detection_value = self.random.random()
			occupancy_size  = 1.0 / 8 # The 9 grid minus the center center square which is occupied by ego

			s = ego_vehicle_state.s
			# Front of Ego
			if   detection_value < 3 * occupancy_size:
				s += bias

			# Back of Ego
			elif detection_value >= 5 * occupancy_size:
				s -= bias

			d = ego_vehicle_state.d
			# Left of Ego
			if   detection_value <  1 * occupancy_size or \
			     detection_value >= 3 * occupancy_size and detection_value < 4 * occupancy_size or \
			     detection_value >= 5 * occupancy_size and detection_value < 6 * occupancy_size:
				if traffic_state.lane_config._left_lane is None:
					left_lane_d = traffic_state.lane_config.left_bound
				else:
					left_lane_d = -traffic_state.lane_config._left_lane.right_bound
				
				d += traffic_state.lane_config.left_bound + left_lane_d

			# Right of Ego
			elif detection_value >= 2 * occupancy_size and detection_value < 3 * occupancy_size or \
				 detection_value >= 4 * occupancy_size and detection_value < 5 * occupancy_size or \
				 detection_value >= 7 * occupancy_size:
				if traffic_state.lane_config._right_lane is None:
					right_lane_d = traffic_state.lane_config.right_bound
				else:
					right_lane_d = -traffic_state.lane_config._right_lane.left_bound

				d += traffic_state.lane_config.right_bound + right_lane_d

			s_start = 0
			x, y = frenet_to_sim_position(reference_line, s, d, s_start)

			# The rest of the values are just cloning the ego vehicle
			hallucinated_vehicle = SDV()
			# Add Position Noise
			# Add Heading  Noise
			self.hallucination_status[self.hallucination_index] = {
				"offsets": {
					"s": s,
					"d": d
				},
				"retention": retention,
				"vehicle":   hallucinated_vehicle
			}
			self.hallucination_index -= 1
				

	def hallucinate_splitting_bounding_box(self, ego_vehicle_state, vid, other_vehicle, traffic_state, sdv_route):
		detection_value = self.random.random()
		likelihood      = self.hallucination_weight
		retention       = 1.0

		if detection_value < likelihood:

			# Initially, we are just cloning the other vehicle, but the gaussian noise are going to discriminate them
			hallucinated_vehicle = Vehicle(self.hallucination_index)
			hallucinated_vehicle.state.set_state_vector(other_vehicle.state.get_state_vector())
			hallucinated_vehicle, has_frenet_location = self.apply_positional_noise(
				ego_vehicle_state, self.hallucination_index, hallucinated_vehicle, sdv_route
			)
			hallucinated_vehicle = self.apply_yaw_noise(ego_vehicle_state, hallucinated_vehicle)

			if has_frenet_location:
				traffic_state.traffic_vehicles[vid]     = hallucinated_vehicle
			else:
				traffic_state.traffic_vehicles_orp[vid] = hallucinated_vehicle


			self.hallucination_status[self.hallucination_index] = {
				"offsets": {
					"x": other_vehicle.state.x - hallucinated_vehicle.state.x,
					"y": other_vehicle.state.y - hallucinated_vehicle.state.y,
					"z": other_vehicle.state.z - hallucinated_vehicle.state.z
				},
				"reference_vehicle": vid,
				"retention":         retention,
				"vehicle":           hallucinated_vehicle
			}
			self.hallucination_index -= 1


	def is_detecting_that_vehicle(self, ego_vehicle_state, other_vehicle):
		distance = self.distance(ego_vehicle_state, other_vehicle)

		is_detected = distance <= self.detection_range_in_meters

		if self.missed_detection_weight == None:
			return is_detected

		if is_detected:
			detection_value     = self.random.random()
			detection_threshold = distance / self.detection_range_in_meters

			is_detected = detection_value <= max(1 - self.missed_detection_weight, 1 - detection_threshold)

		return is_detected

	def update_hallucination(self, ego_vehicle_state, traffic_state, sdv_route):
		for vid, hallucination in list(self.hallucination_status.items()):
			detection_value = self.random.random()

			if hallucination['retention'] < detection_value:
				del self.hallucination_status[vid]
			else:
				self.hallucination_status[vid]['retention'] *= self.hallucination_retention

				# Imitate the reference vehicle
				if 'reference_vehicle' in hallucination:

					# The reference vehicle is out of sight and not a missed detection
					if hallucination['reference_vehicle'] not in self.all_vehicles:
						del self.hallucination_status[vid]
					
					reference_vehicle    = self.all_vehicles[hallucination['reference_vehicle']]
					hallucinated_vehicle = hallucination['vehicle']
					hallucinated_vehicle.state.set_state_vector(reference_vehicle.state.get_state_vector())
					hallucinated_vehicle.state.x += hallucination['offsets']['x']
					hallucinated_vehicle.state.y += hallucination['offsets']['y']
					hallucinated_vehicle.state.z += hallucination['offsets']['z']
					hallucinated_vehicle, has_frenet_location = self.apply_positional_noise(
						ego_vehicle_state, vid, hallucinated_vehicle, sdv_route, tracked_hallucination=True
					)
					hallucinated_vehicle = self.apply_yaw_noise(ego_vehicle_state, hallucinated_vehicle, intensities=[1, 15, 25])

					self.hallucination_status[vid]['offsets'] = {
						"x": reference_vehicle.state.x - hallucinated_vehicle.state.x,
						"y": reference_vehicle.state.y - hallucinated_vehicle.state.y,
						"z": reference_vehicle.state.z - hallucinated_vehicle.state.z
					}

				# Imitate Ego	
				else:
					pass

				if has_frenet_location:
					traffic_state.traffic_vehicles[vid]     = hallucinated_vehicle
				else:
					traffic_state.traffic_vehicles_orp[vid] = hallucinated_vehicle