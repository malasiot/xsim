from client import SimulationClient
import random
import numpy as np
import math

class Env:
	def __init__(self, config):
		self.goal_pos = config.target_pos
		self.goal_radius = config.target_radius
		self.goal_id = config.target
		self.obstacle_box_center = [0.0, 0.6]
		self.obstacle_box_hsz = [0.2, 0.2]
  
		response = self.req("info")
		self.action_size = response["actions"]
		self.state_size = response["boxes"] * 4
		self.goal_size = 4
		self.dense_reward = config.dense_reward

		initial_state = response['state']
		self.goal_orig = self.makeGoal(self.goal_pos, self.obstacle_box_center)
	
  
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
		return response["state"], response['feasible']
	
	#Step simulation and return a tuple (new_state, done)		
	def step(self, action):
		response = self.req("step", {"action": action})
		new_state = response["state"]
		done = response["done"]
		feasible = response['feasible']
				
		return new_state, done, feasible

	def makeGoal(self, target, box) :
		a = np.array([target[0], target[1], box[0], box[1]])
		return a
    
     
	def goal(self):
		return self.goal_orig

  			
	def	to_goal(self, state):
		target = []
		center = [0, 0]
		for b in state['boxes']:
			if b["name"] == self.goal_id:
				target = [b['x'], b['y']]
			else:
				center[0] += b['x']
				center[1] += b['y']
		center[0] /= len(state['boxes'])-1
		center[1] /= len(state['boxes'])-1

		return self.makeGoal(target, center)
		
	def bbox(self, state):
		minx, miny, maxx, maxy = None, None, None, None
		for box in state['boxes']:
			if box['name'] != self.goal_id:
				x, y  = box['x'], box['y']
				minx = x if minx == None else min(x, minx)
				miny = y if miny == None else min(y, miny)
				maxx = x if maxx == None else max(x, maxx)
				maxy = y if maxy == None else max(y, maxy)
    
		return [minx, miny, maxx, maxy]
		
	def computeReward(self, state, goal, done):
		if done:
			return -1, False

		sgoal = self.to_goal(state)
  
		d0 = np.linalg.norm(sgoal[:2] - goal[:2])
		success = d0 < self.goal_radius

		minx, miny, maxx, maxy = self.bbox(state)
		 
  			
		if (minx < goal[2] - self.obstacle_box_hsz[0]) or \
      		(miny < goal[3] - self.obstacle_box_hsz[1]) or \
          	( maxx > goal[2] + self.obstacle_box_hsz[0]) or \
            ( maxy > goal[3] + self.obstacle_box_hsz[1]):
			success = False 
		
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
