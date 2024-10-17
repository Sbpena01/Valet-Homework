from DelieveryBot import *
from Environment import *
import numpy as np
from queue import PriorityQueue

class State:
    def __init__(self, position: tuple, angle, v, omega, parent=None):
        self.x = position[0]
        self.y = position[1]
        self.theta = angle
        self.v = v
        self.omega = omega
        self.parent = parent
        self.cost = 10000
        
    def __str__(self):
        return f"Position: {self.x}, {self.y} Cost: {self.cost}"

class Search:
    def __init__(self, robot: DelieveryBot, goal: State, environment: Environment):
        self.robot = robot
        self.map = environment
        self.search_queue = PriorityQueue()
        self.start = State((robot.x, robot.y), robot.theta, robot.v, robot.steering_angle)
        self.goal = goal
        self.max_count = 3000
        self.dt = 0.05
    
    def search(self, debug=False):
        self.search_queue.put((0, 0, self.start))
        count = 0
        hash_count = 0
        visited = list()
        visited_states = list()
        while not self.search_queue.empty() != 0 and count < self.max_count:
            _, _, current_state = self.search_queue.get()
            
            if self.checkGoal(current_state):
                print("Found a path!")
                if debug:
                    return visited_states
                return self.getPath(current_state)
            current_pixel = (int(current_state.x), int(current_state.y))
            
            if current_pixel in visited:
                continue
            visited.append(current_pixel)
            visited_states.append(current_state)
            count += 1
            actions = self.actions()
            for action in actions:
                result = self.results(current_state, action)
                if result is None:
                    continue
                if not self.robot.checkCollision(self.map, (result.x, result.y), result.theta):
                    result.cost = self.calculateCost(current_state, result)
                    self.search_queue.put((result.cost, hash_count, result))
                    hash_count += 1
        
        if(count >= self.max_count):
            print("Max Count Exceeded")
        if debug:
            return visited_states
        return None
        
    def getPath(self, state: State):
        path = list()
        while state.parent is not None:
            path.append(state)
            state = state.parent
        path.reverse()
        return path
    
    def checkGoal(self, state: State):
        return state.cost <= 15
    
    def actions(self):
        steering_angles = [-30, -20, -10, 0, 10, 20, 30]
        velocities = [-self.robot.v, self.robot.v]
        action_list = list()
        for angle in steering_angles:
            for v in velocities:
                action_list.append((v, angle))
        return action_list
    
    def results(self, state: State, action):
        x, y, theta, v, omega = self.robot.kinematics(self.dt, action[0], action[1], state.x, state.y, state.theta)
        return State((x,y), theta, v, omega, state)

    def calculateCost(self, prev_state: State, new_state: State):
        # We want to avoid turning, so increase the cost if the wheel velocities are different.
        distance = self.heuristic(new_state)
        omega_term = 0 * np.abs(new_state.omega)
        cost = distance + omega_term
        return cost
    
    def heuristic(self, state: State):
        distance = np.sqrt((self.goal.x - state.x)**2 + (self.goal.x - state.y)**2)
        return distance
    