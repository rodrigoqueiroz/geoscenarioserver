from math import sqrt

from sv.planners.ruleEngine.constants import KMH_TO_MS
from sv.planners.ruleEngine.Extractor import Extractor
from util.Utils import distance_2p

class ActorExtractor(Extractor):
	def __init__(self, name):
		self.name = name

	def extract_actor(self, actor, closest_id, traffic_state):
		distance          = None
		ego_state         = traffic_state.vehicle_state
		speed             = max(actor.state.s_vel, 0.0)
		time_to_collision = None

		if speed == 0.0:
			speed = sqrt(pow(actor.state.x_vel, 2) + pow(actor.state.y_vel, 2))

		# Frenet Frame localizable
		if actor.state.s != 0.0 or actor.state.d != 0.0:
			distance = actor.state.s - ego_state.s

		# Euclidian localizable
		else:
			distance = distance_2p(actor.state.x, actor.state.y, ego_state.x, ego_state.y)

		ego_pessimistic_speed = max(2.0, ego_state.s_vel)

		if ego_pessimistic_speed > speed:
			time_to_collision = distance / (ego_pessimistic_speed - speed)

		return {
			"d":        actor.state.d,
			"d_vel":    actor.state.d_vel,
			"d_acc":    actor.state.d_acc,
			"s":        actor.state.s,
			"s_vel":    actor.state.s_vel,
			"s_acc":    actor.state.s_acc,
			"x":        actor.state.x,
			"x_vel":    actor.state.x_vel,
			"x_acc":    actor.state.x_acc,
			"y":        actor.state.y,
			"y_vel":    actor.state.y_vel,
			"y_acc":    actor.state.y_acc,
			"distance":          distance,
			"is_leading":		 actor.id == closest_id,
			"speed":             speed * KMH_TO_MS,
			"time_to_collision": time_to_collision
		}

	def extract_actors(self, actors, closest_id, traffic_state):
		workspace = {}

		for actor in actors:
			workspace[self.name[0] + '_' + str(actor.id)] = self.extract_actor(actor, closest_id, traffic_state)

		return workspace

	def generate_features(self, traffic_state):
		closest_actor = traffic_state.road_occupancy.front_center.closest()
		closest_id    = closest_actor.id if closest_actor else None

		return {
			"front_center": self.extract_actors(getattr(traffic_state.road_occupancy.front_center, self.name), closest_id, traffic_state)
		}