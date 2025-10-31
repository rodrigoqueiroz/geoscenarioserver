from geoscenarioserver.sv.planners.ruleEngine.Extractor import Extractor

class TravelExtractor(Extractor):
	def __init__(self):
		self.name = "travel"

	def extract_goal(self, traffic_state):		
		if not traffic_state.route_complete:
			return None

		return self.to_frenet_frame(traffic_state.goal_point_frenet)

	def extract_lane(self, lane_config, relationship = None):
		return {
			"max_velocity": lane_config.max_velocity,
			"relationship": relationship
		}

	def extract_lanes(self, lane_config):
		lanes      = {
			"current": self.extract_lane(lane_config)
		}

		def aggregate_all_lanes(lane_config, side, relation, uuid):
			iterator  = 0
			lane      = lane_config
			workspace = {}

			while lane != None:
				iterator += 1
				next_lane = getattr(lane, side)

				if next_lane != None:
					workspace[uuid + str(iterator)] = self.extract_lane(next_lane, getattr(lane, relation))

				lane = next_lane

			return workspace

		lanes["left"]  = aggregate_all_lanes(lane_config, "_left_lane",  "_left_relationship",  "l_")
		lanes["right"] = aggregate_all_lanes(lane_config, "_right_lane", "_right_relationship", "r_")

		return lanes

	def generate_features(self, traffic_state):
		return {
			"goal":  self.extract_goal(traffic_state),
			"lanes": self.extract_lanes(traffic_state.lane_config)
		}