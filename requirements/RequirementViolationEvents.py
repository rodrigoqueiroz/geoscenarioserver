import json
import os

from multiprocessing import Manager, Value

manager = Manager()

# Singleton
agent_collisions = manager.dict()
agent_ticks      = manager.dict()
file_name        = os.getenv('VIOLATION_REPORT_FOLDER', './results') + '/violations.json'
global_tick      = Value('i', -1)
violations       = manager.dict()

# Generic
class ScenarioEnd:
	def __init__(self):
		os.makedirs(os.path.dirname(file_name), exist_ok=True)
		with open(file_name, "w+") as file:
			file.write(json.dumps(violations.copy()))


class UnmetRequirement:
	def raise_it(self, agent_id, payload):
		agent_logs = violations[agent_id]
		agent_tick = agent_ticks[agent_id]

		if agent_tick not in agent_logs:
			agent_logs[agent_tick] = {}

		agent_logs[agent_tick][self.__class__.__name__] = payload
		
		# That reassignment is necessary for the update to work in multiprocessing
		violations[agent_id] = agent_logs
		

# Specific
class AgentTick:
	def __init__(self, agent_id):
		if agent_id not in agent_ticks:
			agent_collisions[agent_id] = {}
			agent_ticks[agent_id]      = -1
			violations[agent_id]       = {}

		agent_ticks[agent_id] += 1


class CollisionWithVehicle(UnmetRequirement):
	def __init__(self, agent_id, vid):
		collision_state = agent_collisions[agent_id]

		# There must be a gap of 5 ticks without collision between collision with the same agent
		if vid not in collision_state or agent_ticks[agent_id] - 5 > collision_state[vid]:
			self.raise_it(agent_id, {
				'colliderId': vid,
				'message': 'v' + str(agent_id) + ' bounding box overlapped with the vehicle agent v' + str(vid)
			})

		collision_state[vid] = agent_ticks[agent_id]

		# That reassignment is necessary for the update to work in multiprocessing
		agent_collisions[agent_id] = collision_state

class GlobalTick:
	def __init__(self):
		global global_tick
		with global_tick.get_lock():
			global_tick.value += 1


class GoalOvershot(UnmetRequirement):
	def __init__(self, agent_id):
		self.raise_it(agent_id, {
			'message': 'v' + str(agent_id) + ' drove past its target location'
		})


class ScenarioCompletion(Exception):
    pass


class ScenarioTimeout(UnmetRequirement):
	def __init__(self, timeout):
		for agent_id in agent_ticks:
			self.raise_it(agent_id, {
				'message': 'v' + str(agent_id) + ' did not reach its target location during the ' 
				    		   + str(timeout) + ' seconds allowed by this scenario.'
			})
		ScenarioEnd()

# Autoclean
if os.path.exists(file_name):
	os.remove(file_name)