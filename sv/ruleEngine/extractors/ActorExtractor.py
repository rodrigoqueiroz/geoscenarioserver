from math import sqrt

from sv.ruleEngine.Extractor import Extractor

class ActorExtractor(Extractor):
	def __init__(self, name):
		self.name = name

	def extract_actor(self, actor):
		speed = actor.state.s_vel

		if speed == 0.0:
			speed = sqrt(pow(actor.state.x_vel, 2) + pow(actor.state.y_vel, 2))

		return {
			"speed": speed
		}

	def extract_actors(self, actors):
		workspace = {}

		for actor in actors:
			workspace[self.name[0] + '_' + str(actor.id)] = self.extract_actor(actor)

		return workspace

	def generate_features(self, traffic_state):
		return {
			"front_center": self.extract_actors(getattr(traffic_state.road_occupancy.front_center, self.name))
		}