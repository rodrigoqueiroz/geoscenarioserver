import json
import os

from multiprocessing import Manager, Value

from SimConfig import TRAFFIC_RATE

manager = Manager()

# Singleton
agent_collisions = manager.dict()
agent_ticks      = manager.dict()
file_name        = os.path.join(os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs")), 
                                "violations.json")
global_tick      = Value('i', -1)
violations       = manager.dict()

# Constants
TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR = 5

# Generic
class ScenarioEnd:
	def __init__(self):
		print('Scenario Ended in {} seconds'.format(global_tick.value / TRAFFIC_RATE))

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

class BrokenScenario(UnmetRequirement):
	def __init__(self, agent_id, message):
		self.raise_it(agent_id, {
			'message': 'v' + str(agent_id) + ' ' + message
		})
		ScenarioEnd()

class CollisionWithPedestrian(UnmetRequirement):
	def __init__(self, agent_id, pid, collision_zone, relative_angle):
		collision_state = agent_collisions[agent_id]
	

		if pid not in collision_state or agent_ticks[agent_id] - TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR > collision_state[pid]:
			alias_name  = 'p_' + str(pid)
			myself_name = 'v_' + str(agent_id)

			self.raise_it(agent_id, {
				'angle': relative_angle,
				'collections': {
					'pedestrians': {
						'alias': alias_name,
						'id':    pid,
					}
				},
				'message': f'{myself_name} bounding box overlapped with the pedestrian agent {alias_name} on the vehicle {collision_zone} side at {relative_angle} degrees',
				'zone' : collision_zone
			})

		collision_state[pid] = agent_ticks[agent_id]

		# That reassignment is necessary for the update to work in multiprocessing
		agent_collisions[agent_id] = collision_state

class CollisionWithVehicle(UnmetRequirement):
	def __init__(self, agent_id, vid):
		collision_state = agent_collisions[agent_id]

		# There must be a gap of 5 ticks without collision between collision with the same agent
		if vid not in collision_state or agent_ticks[agent_id] - TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR > collision_state[vid]:
			alias_name  = 'v_' + str(vid)
			myself_name = 'v_' + str(agent_id)

			self.raise_it(agent_id, {
				'collections': {
					'vehicles': {
						'alias': alias_name,
						'id':    vid
					}
				},
				'message': myself_name + ' bounding box overlapped with the vehicle agent ' + alias_name
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