import math
import numpy as np

from dash.DashboardSharedMemory import get_center_id, set_vehicles
from sv.SDVTrafficState         import fill_occupancy
from util.Transformations       import (OutsideRefPathException, sim_to_frenet_frame)

class Perception:
	def __init__(self, vid, detection_range_in_meters, misdetection_weight, noise_position_mixture, 
				 noise_yaw_mostly_reliable, noise_yaw_strongly_innacurate, seed=1):
		self.detection_range_in_meters = detection_range_in_meters
		self.rng_tick                  = 0
		self.misdetection_weight       = misdetection_weight
		self.my_vid                    = vid
		self.position_mean             = noise_position_mixture[0]
		self.position_std              = noise_position_mixture[1]
		self.random                    = np.random
		self.vehicles                  = {}
		self.yaw_mostly_reliable       = noise_yaw_mostly_reliable
		self.yaw_strongly_innacurate   = noise_yaw_strongly_innacurate

		self.random.seed(vid + seed)

	def apply_noise(self, traffic_state, lanelet_map, sdv_route):

		if self.detection_range_in_meters == None:
			return traffic_state

		for vid, vehicle in list(traffic_state.traffic_vehicles.items()):
			if not self.is_detecting_that_vehicle(traffic_state.vehicle_state, vehicle):
				del traffic_state.traffic_vehicles[vid]
			else:
				traffic_state.traffic_vehicles[vid] = self.apply_positional_noise(traffic_state.vehicle_state, vid, vehicle, sdv_route)
				traffic_state.traffic_vehicles[vid] = self.apply_yaw_noise(traffic_state.vehicle_state, vid, vehicle)

		for vid, vehicle in list(traffic_state.traffic_vehicles_orp.items()):
			if not self.is_detecting_that_vehicle(traffic_state.vehicle_state, vehicle):
				del traffic_state.traffic_vehicles_orp[vid]
			else:
				traffic_state.traffic_vehicles_orp[vid] = self.apply_positional_noise(traffic_state.vehicle_state, vid, vehicle, sdv_route)
				traffic_state.traffic_vehicles_orp[vid] = self.apply_yaw_noise(traffic_state.vehicle_state, vid, vehicle)

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

	def apply_positional_noise(self, ego_vehicle_state, vid, vehicle, sdv_route):
		distance  = self.distance(ego_vehicle_state, vehicle)
		intensity = distance / self.detection_range_in_meters
		mixture   = [ self.position_mean, self.position_std ]

		vehicle.state.x = self.apply_gaussian_noise(intensity, vehicle.state.x, mixture)
		vehicle.state.y = self.apply_gaussian_noise(intensity, vehicle.state.y, mixture)
		vehicle.state.z = self.apply_gaussian_noise(intensity, vehicle.state.z, mixture)

		if vid not in self.vehicles:
			vehicle.state.x_vel = 0
			vehicle.state.y_vel = 0
			vehicle.state.z_vel = 0
			vehicle.state.x_acc = 0
			vehicle.state.y_acc = 0
			vehicle.state.z_acc = 0

			vehicle.state.s_vel = 0
			vehicle.state.d_vel = 0
			vehicle.state.s_acc = 0
			vehicle.state.d_acc = 0
		else:
			vehicle.state.x_vel = vehicle.state.x     - self.vehicles[vid].state.x
			vehicle.state.y_vel = vehicle.state.y     - self.vehicles[vid].state.y
			vehicle.state.z_vel = vehicle.state.z     - self.vehicles[vid].state.z
			vehicle.state.x_acc = vehicle.state.x_vel - self.vehicles[vid].state.x_vel
			vehicle.state.y_acc = vehicle.state.y_vel - self.vehicles[vid].state.y_vel
			vehicle.state.z_acc = vehicle.state.z_vel - self.vehicles[vid].state.z_vel

			s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_global_path(), vehicle.state.get_X(), vehicle.state.get_Y(), 0)
			vehicle.state.set_S(s_vector)
			vehicle.state.set_D(d_vector)

		return vehicle

	def apply_yaw_noise(self, traffic_state, vid, vehicle):
		return vehicle

	def distance(self, ego_vehicle_state, other_vehicle):
		return math.sqrt(
			pow(ego_vehicle_state.x - other_vehicle.state.x, 2) +
			pow(ego_vehicle_state.y - other_vehicle.state.y, 2) +
			pow(ego_vehicle_state.z - other_vehicle.state.z, 2)
		)

	def is_detecting_that_vehicle(self, ego_vehicle_state, other_vehicle):
		distance = self.distance(ego_vehicle_state, other_vehicle)

		is_detected = distance <= self.detection_range_in_meters

		if self.misdetection_weight == None:
			return is_detected

		if is_detected:
			detection_value     = self.random.random()
			detection_threshold = distance / self.detection_range_in_meters

			is_detected = detection_value <= max(1 - self.misdetection_weight, 1 - detection_threshold)

		return is_detected
