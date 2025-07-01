def to_pid(agent_id):
	return 'p_' + str(agent_id)

def to_vid(agent_id):
	return 'v_' + str(agent_id)

class AgentTick:
	def __init__(self, agent_id):
		self.agent_id = agent_id


class BrokenScenario:
	def __init__(self, agent_id, message):
		self.agent_id = agent_id
		self.payload  = {
			'message': to_vid(agent_id) + ' ' + message
		}


class CollisionWithPedestrian:
	def __init__(self, agent_id, pid, collision_zone, relative_angle):
		alias_name    = to_pid(pid)
		myself_name   = to_vid(agent_id)

		self.agent_id = agent_id
		self.payload  = {
			'angle': relative_angle,
			'collections': {
				'pedestrians': {
					'alias': alias_name,
					'id':    pid,
				}
			},
			'message': f'{myself_name} bounding box overlapped with the pedestrian agent {alias_name} on the vehicle {collision_zone} side at {relative_angle} degrees',
			'zone' : collision_zone
		}
		self.uid      = pid


class CollisionWithVehicle:
	def __init__(self, agent_id, vid):
		alias_name    = to_vid(vid)
		myself_name   = to_vid(agent_id)

		self.agent_id = agent_id
		self.payload  = {
			'collections': {
				'vehicles': {
					'alias': alias_name,
					'id':    vid
				}
			},
			'message': myself_name + ' bounding box overlapped with the vehicle agent ' + alias_name
		}
		self.uid      = vid


class ElapsedTime:
	def __init__(self, elapsed_time):
		self.elapsed_time = elapsed_time


class GoalOvershot:
	def __init__(self, agent_id):
		self.agent_id = agent_id
		self.payload  = {
			'message': to_vid(agent_id) + ' drove past its target location'
		}


class ScenarioCompletion(Exception):
    pass


class ScenarioEnd:
	def __init__(self, agent_id):
		self.agent_id = agent_id


class ScenarioInterrupted:
	def __init__(self, agent_id):
		self.agent_id = agent_id
		self.payload  = {
			'agentId': agent_id,
			'message': to_vid(agent_id) + ' scenario interrupted by the user'
		}
		

class ScenarioTimeout:
	def __init__(self, timeout):
		self.prior     = 'v_'
		self.posterior = ' did not reach its target location during the ' + str(timeout) +\
			             ' seconds allowed by this scenario.'


class SoftRequirement:
	def __init__(self, agent_id, req_name, req_value):
		self.agent_id  = agent_id
		self.req_name  = req_name
		self.req_value = req_value