import numpy as np

from math import cos, radians, sin

from Actor import ActorSimState
from requirements.RequirementViolationEvents import CollisionWithVehicle, GoalOvershot, ScenarioCompletion, ScenarioEnd
from sv.SDVTrafficState import *

class RequirementsChecker:
	def __init__(self, ego_vehicle, goal_ends_simulation):
		self.conditions = [
			self.detect_collisions,
			self.detect_goal_overshot
		]

		self.ego_vehicle = ego_vehicle
		self.goal_ends_simulation = goal_ends_simulation
		self.goal_s_distance = float('inf')
		self.vehicles = {}

	def analyze(self, traffic_state:TrafficState):
		self.vehicles = {}
		self.vehicles.update(traffic_state.traffic_vehicles)
		self.vehicles.update(traffic_state.traffic_vehicles_orp)

		for condition in self.conditions:
			condition(traffic_state)

	def calculate_rectangular_bounding_box(self, vehicle):
		center_x     = vehicle.state.x
		center_y     = vehicle.state.y
		half_length  = vehicle.bounding_box_length / 2
		half_width   = vehicle.bounding_box_width  / 2
		yaw          = -vehicle.state.yaw

		bottom_left  = self.rotate(center_x, center_y, center_x - half_length, center_y - half_width, yaw)
		bottom_right = self.rotate(center_x, center_y, center_x + half_length, center_y - half_width, yaw)
		top_left     = self.rotate(center_x, center_y, center_x - half_length, center_y + half_width, yaw)
		top_right    = self.rotate(center_x, center_y, center_x + half_length, center_y + half_width, yaw)

		return np.array([top_left, top_right, bottom_right, bottom_left])

	def detect_collisions(self, traffic_state):
		ego_vehicle       = self.ego_vehicle

		if ego_vehicle.sim_state is not ActorSimState.ACTIVE:
			return

		# The reference above is on another thread and has not been updated
		ego_vehicle.state = traffic_state.vehicle_state
		ego_box           = self.calculate_rectangular_bounding_box(ego_vehicle)

		min_x = (ego_vehicle.state.x - ego_vehicle.radius)
		max_x = (ego_vehicle.state.x + ego_vehicle.radius)
		min_y = (ego_vehicle.state.y - ego_vehicle.radius)
		max_y = (ego_vehicle.state.y + ego_vehicle.radius)

		for vid, vehicle in self.vehicles.items():
			if vehicle.sim_state not in [ ActorSimState.ACTIVE, ActorSimState.ACTIVE.value]:
				continue

			# To avoid O(v²) comparisons and favor O(v) comparisons whenever possible
			if  (min_x <= vehicle.state.x <= max_x) and (min_y <= vehicle.state.y <= max_y):
				vehicle_box = self.calculate_rectangular_bounding_box(vehicle)
				if self.do_polygons_intersect(ego_box, vehicle_box):
					CollisionWithVehicle(traffic_state.vid, vid)


	def detect_goal_overshot(self, traffic_state:TrafficState):
		""" Checks if the vehicle has reached or passed the goal point in the frenet frame.
		"""

		if traffic_state.goal_point_frenet is None:
			return

		ego_s = traffic_state.vehicle_state.s
		ego_d = traffic_state.vehicle_state.d

		# max meters away from target
		goal_s_distance = 4.0
		goal_d_distance = 2.0

		goal_s = traffic_state.goal_point_frenet[0]
		goal_d = traffic_state.goal_point_frenet[1]

		s_convergence = abs(goal_s - ego_s)
		d_convergence = abs(goal_d - ego_d)

		if traffic_state.route_complete and \
		   s_convergence < goal_s_distance and \
		   d_convergence < goal_d_distance:

		   # Let the vehicle get closer to its destination...
			if s_convergence < self.goal_s_distance:
				self.goal_s_distance = s_convergence
				return

			if traffic_state.vehicle_state.s_vel > 1.0:
				GoalOvershot(traffic_state.vid)
			
			if self.goal_ends_simulation:
				ScenarioEnd()
				raise ScenarioCompletion()

	def do_polygons_intersect(self, polygon_a, polygon_b):
		"""
		* Reference: https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles
		* Determine whether there is an intersection between the two polygons described
		* by the lists of vertices. Uses the Separating Axis Theorem
		*
		* @param a an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
		* @param b an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
		* @return true if there is any intersection between the 2 polygons, false otherwise
		"""
		polygons = [polygon_a, polygon_b];

		for polygon in polygons:
			# for each polygon, look at each edge of the polygon, and determine if it separates the two shapes
			for edge_index in range(len(polygon)):

				# grab 2 vertices to create an edge
				edge_index_2 = (edge_index + 1) % len(polygon);
				[ x1, y1 ] = polygon[edge_index];
				[ x2, y2 ] = polygon[edge_index_2];

				# find the line perpendicular to this edge
				normal = { 'x': y2 - y1, 'y': x1 - x2 };

				minA, maxA = None, None
				# for each vertex in the first shape, project it onto the line perpendicular to the edge
				# and keep track of the min and max of these values
				for vertex in polygon_a:
					[vx, vy] = vertex
					projected = normal['x'] * vx + normal['y'] * vy;
					
					if (minA is None) or (projected < minA):
						minA = projected

					if (maxA is None) or (projected > maxA):
						maxA = projected

				# for each vertex in the second shape, project it onto the line perpendicular to the edge
				# and keep track of the min and max of these values
				minB, maxB = None, None
				for vertex in polygon_b:
					[vx, vy] = vertex
					projected = normal['x'] * vx + normal['y'] * vy

					if (minB is None) or (projected < minB):
						minB = projected

					if (maxB is None) or (projected > maxB):
						maxB = projected

				# if there is no overlap between the projects, the edge we are looking at separates the two
				# polygons, and we know there is no overlap
				if (maxA < minB) or (maxB < minA):
					return False;

		return True

	def rotate(self, center_x, center_y, x, y, degree_theta):
		radian_theta = radians(degree_theta)
		x2 = center_x + (x - center_x) * cos(radian_theta) + (y - center_y) * sin(radian_theta)
		y2 = center_y - (x - center_x) * sin(radian_theta) + (y - center_y) * cos(radian_theta)

		return np.array([ x2, y2 ])