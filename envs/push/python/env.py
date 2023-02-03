from client import SimulationClient
import random
import numpy as np
import math

class Env:
	def __init__(self, config):
		self.goal_pos = config.target_pos
		self.goal_radius = config.target_radius
		self.goal_id = config.target
		self.obstacle_radius = 0.05
  
		response = self.req("info")
		self.action_size = response["actions"]
		self.state_size = response["boxes"] * 4
		self.goal_size = response["boxes"] * 2
		self.dense_reward = config.dense_reward

		initial_state = response['state']
		self.goal_orig = self.makeGoal(initial_state, self.goal_pos)
	
  
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

	def makeGoal(self, state, target) :
		a = np.zeros(self.goal_size)
		k=0
		a[k] = target[0] ; k+=1
		a[k] = target[1] ; k+=1
		
		for b in state['boxes']:
			if b["name"] != self.goal_id:
				a[k] = b['x'] ; k+=1
				a[k] = b['y'] ; k+=1
				
		return a
    
     
	def goal(self):
		return self.goal_orig

  			
	def	to_goal(self, state):
		target = []
		for b in state['boxes']:
			if b["name"] == self.goal_id:
				target = [b['x'], b['y']]
				break

		return self.makeGoal(state, target)
		
	def computeReward(self, state, goal, done):
		if done:
			return -10, False

		sgoal = self.to_goal(state)
  
		d0 = np.linalg.norm(sgoal[:2] - goal[:2])
		success = d0 < self.goal_radius

		for k in range(2, goal.size, 2):
			d = np.linalg.norm(sgoal[k:k+2] - goal[k:k+2])
			if d > self.obstacle_radius:
				success = False 
				d0 += d ;

		d0 /=  goal.size/2
		
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
			a[k] = b["x"] * 0.1 ; k = k+1 ;
			a[k] = b["y"] * 0.1 ; k = k+1 ;
			a[k] = math.sin(b["theta"]); k = k+1 ;
			a[k] = math.cos(b["theta"]); k = k+1 ;
		return a 

	def goalVec(self, goal):
		return goal * 0.1 ;
		
	
	
#done = False

#env = Env() ;
#state, info = env.reset() ;

#while not done:
#	action = random.randint(0, info["actions"]-1)	
#	new_state, done = env.step(action)
