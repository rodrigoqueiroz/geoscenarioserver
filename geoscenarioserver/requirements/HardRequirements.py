import numpy as np

from math import sin

from geoscenarioserver.Actor import ActorSimState
from geoscenarioserver.requirements.RequirementViolationEvents import CollisionWithVehicle, CollisionWithPedestrian, GoalOvershot, ScenarioCompletion, ScenarioEnd
from geoscenarioserver.sv.SDVTrafficState import TrafficState
from geoscenarioserver.util.BoundingBoxes import calculate_rectangular_bounding_box

class HardRequirements:
	def __init__(self, ego_vehicle, goal_ends_simulation):
		self.ego_vehicle          = ego_vehicle
		self.goal_ends_simulation = goal_ends_simulation
		self.goal_s_distance      = float('inf')

	def all_conditions(self):
		return [
			self.detect_collisions,
			self.detect_goal_overshot
		]


	def collision_check(self, vid, pid, centre, vehicle, radius):
		#take the middle of the vehicle and project an arc with a radius half the length of the car to the front of the car
		#the front of the car can be determined by the yaw
		center_x     = vehicle.state.x
		center_y     = vehicle.state.y
		arc_radius  = vehicle.length / 2 # vehicle half length
		half_width   = vehicle.width  / 2
		yaw          = np.radians(vehicle.state.yaw)
		
		ped_x, ped_y = centre
		ped_rad = radius
		theta_max = np.arcsin((half_width)/arc_radius)

		#first check if position of pedestrian is even close to the vehicle
		outer_rad = arc_radius/np.cos(theta_max)
		car_to_ped_dist = np.sqrt((center_x - ped_x) ** 2 + (center_y - ped_y) ** 2)

		collision_vector = np.array([ped_x - center_x, ped_y - center_y])
		vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])
		dot_product = np.dot(vehicle_direction, collision_vector)
		denom = np.linalg.norm(collision_vector)*np.linalg.norm(vehicle_direction)
		relative_angle = np.arccos(dot_product/denom)
		collision_zone = None

		if car_to_ped_dist < outer_rad:
			#first do circle check
			distance = round(np.sqrt((center_x - ped_x) ** 2 + (center_y - ped_y) ** 2), 2)
			
			if distance <= round(arc_radius + ped_rad, 2):
				#we know there is a possible collision. check if its front
				collision_vector = np.array([ped_x - center_x, ped_y - center_y])
				vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])
				dot_product = np.dot(vehicle_direction, collision_vector)
				denom = np.sqrt(((collision_vector[0])**2 + (collision_vector[1])**2))*np.sqrt((vehicle_direction[0])**2 + (vehicle_direction[1])**2)

				relative_angle = np.arccos(dot_product/denom)
				cross_product = vehicle_direction[0] * collision_vector[1] - vehicle_direction[1] * collision_vector[0]

				#check circle for right or left
				if cross_product < 0:
					relative_angle = -relative_angle
				
				theta_max = np.degrees(theta_max)
				relative_angle = round(np.degrees(relative_angle),2)

				#assign a collision zone
				if (-theta_max <= relative_angle <= theta_max):
					collision_zone = "front"
				elif 180 - theta_max <= relative_angle or relative_angle <= -180 + theta_max:
					collision_zone = "rear"
				else:
					if -90 <= relative_angle <= 90:
						angle = 90 - abs(relative_angle)
					else:
						angle = abs(relative_angle) - 90
					boundary_length = round(half_width/np.cos(np.radians(angle)),2)
					if distance <= boundary_length + ped_rad:
						if 0 > relative_angle:
							collision_zone = "right"
						else:
							collision_zone = "left"

		if collision_zone:
			CollisionWithPedestrian(vid, pid, collision_zone, relative_angle)


	def detect_collisions(self, traffic_state:TrafficState):
		ego_vehicle = self.ego_vehicle

		vehicles = {}
		vehicles.update(traffic_state.traffic_vehicles)
		vehicles.update(traffic_state.traffic_vehicles_orp)

		if ego_vehicle.sim_state is not ActorSimState.ACTIVE:
			return

		ego_min_x, ego_min_y = np.min(ego_vehicle.bounding_box, axis=0)
		ego_max_x, ego_max_y = np.max(ego_vehicle.bounding_box, axis=0)

		for vid, vehicle in vehicles.items():
			if vehicle.sim_state not in [ ActorSimState.ACTIVE, ActorSimState.ACTIVE.value]:
				continue

			max_offset = max(vehicle.width / 2, vehicle.length / 2)

			# To avoid O(v²) comparisons and favor O(v) comparisons whenever possible.
			# Suppose big rectangles were overshoting the real dimension of the vehicles,
			# Would they overlap? If yes, then go over the polygons and make the true comparison.
			if (ego_min_x - max_offset <= vehicle.state.x <= ego_max_x + max_offset) and \
			   (ego_min_y - max_offset <= vehicle.state.y <= ego_max_y + max_offset):			   
				vehicle_box = calculate_rectangular_bounding_box(vehicle)

				if self.do_polygons_intersect(ego_vehicle.bounding_box, vehicle_box):
					CollisionWithVehicle(ego_vehicle.id, vid)
				

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
			if 1e-12 < abs(s_convergence - self.goal_s_distance) and \
			   s_convergence < self.goal_s_distance:
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