from client import SimulationClient
import random
import numpy as np
import math

class Env:
	def __init__(self, config):
		self.goal_pos = config.target_pos
		self.goal_radius = config.target_radius
		self.goal_id = config.target
  
		response = self.req("info")
		self.action_size = response["actions"]
		self.state_size = response["boxes"] * 4
		self.goal_size = 2
		self.dense_reward = config.dense_reward
	
  
	def req(self, key, args = {}):
		sock = SimulationClient() ;
		sock.connect()
		params = {"request": key} ;
		params.update(args)
		sock.request(params)
		response = sock.response()
		return response

	# Reset simulation and returns a tuple (state, info)      
	def reset(self): 
		response = self.req("reset")
		return response["state"], response["feasible"]
	
	#Step simulation and return a tuple (new_state, done)		
	def step(self, action):
		response = self.req("step", {"action": action})
		new_state = response["state"]
		done = response["done"]
		feasible = response["feasible"]
		
		return new_state, done, feasible

	def goal(self):
		return np.array(self.goal_pos)

  			
	def	to_goal(self, state):
		for b in state['boxes']:
			if b["name"] == self.goal_id:
				return np.array([b['x'], b['y']])
		
	def computeReward(self, state, goal, done):
		if done:
			return -10, False

		for b in state['boxes']:
			if b["name"] == self.goal_id:
				g = np.array([b['x'], b['y']])
				d = np.linalg.norm(g - goal) 
				break
		success = d < self.goal_radius
		
		if self.dense_reward:
			reward = 1. if success else -d
		else:
            # Sparse reward:
            #    1 => success
            #   -1 => fail
			reward = 1. if success else -1.
   
		return reward, success


	def stateVec(self, state):
		a = np.zeros(self.state_size)
		k = 0
		for b in state["boxes"]:
			a[k] = b["x"] ; k = k+1 ;
			a[k] = b["y"] ; k = k+1 ;
			a[k] = math.sin(b["theta"]); k = k+1 ;
			a[k] = math.cos(b["theta"]); k = k+1 ;
		return a 
		
	
	
#done = False

#env = Env() ;
#state, info = env.reset() ;

#while not done:
#	action = random.randint(0, info["actions"]-1)	
#	new_state, done = env.step(action)
