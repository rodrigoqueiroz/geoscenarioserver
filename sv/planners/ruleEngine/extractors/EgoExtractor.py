from sv.planners.ruleEngine.Extractor import Extractor

class EgoExtractor(Extractor):
	def __init__(self):
		self.name = "ego"

	def generate_features(self, traffic_state):
		return {
			"d":     traffic_state.vehicle_state.d,
			"d_vel": traffic_state.vehicle_state.d_vel,
			"d_acc": traffic_state.vehicle_state.d_acc,
			"s":     traffic_state.vehicle_state.s,
			"s_vel": traffic_state.vehicle_state.s_vel,
			"s_acc": traffic_state.vehicle_state.s_acc,
			"x":     traffic_state.vehicle_state.x,
			"x_vel": traffic_state.vehicle_state.x_vel,
			"x_acc": traffic_state.vehicle_state.x_acc,
			"y":     traffic_state.vehicle_state.y,
			"y_vel": traffic_state.vehicle_state.y_vel,
			"y_acc": traffic_state.vehicle_state.y_acc
		}