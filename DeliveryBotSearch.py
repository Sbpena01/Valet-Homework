from Search import *
from Environment import *
from DeliveryBot import *

import numpy as np
from queue import PriorityQueue

class DeliveryBotSearch(Search):
    # To see docstrings for the methods, see Search.py
    
    def __init__(self, robot: DeliveryBot, goal: State, environment: Environment):
        self.robot = robot
        self.map = environment
        self.search_queue = PriorityQueue()
        self.start = State((robot.x, robot.y), robot.theta, robot.v, robot.steering_angle)
        self.goal = goal
        self.max_count = 6000
        self.dt = 0.25
        self.grid_size = 1  # Pixels
        
    def search(self, debug=False):
        # Initializes the variables and begins the search by adding the starting node to the
        # priority queue
        self.search_queue.put((0, self.start))
        count = 0
        visited = set()
        visited_states = list()
        
        while not self.search_queue.empty() and count < self.max_count:
            _, current_state = self.search_queue.get()  # Gets the state with the lowest cost
            
            # Before attempting to search, check if the current state is the goal state.
            if self.checkGoal(current_state):
                print("Found a path!")
                if debug:
                    return visited_states
                return self.getPath(current_state)
            
            # Create the list of control inputs
            actions = self.actions(current_state)
            
            # Loop through each pair of control inputs and create states
            # based on the results of the kinematic equations
            for action in actions:
                result = self.results(current_state, action)
                if result is None:
                    continue
                grid_position = self.to_grid((result.x, result.y))

                # We want to ensure that the state does not collide with an obstacle
                # and reaches a unique grid cell.
                if self.robot.checkCollision(self.map, (result.x, result.y), result.theta):
                    continue
                if grid_position in visited:
                    continue

                result.cost = self.calculateCost(current_state, result)
                try:
                    self.search_queue.put((result.cost, result))
                    
                # If two states have equal costs, only have one in the queue.
                except TypeError as e:
                    print("Two nodes with the same cost.")
                    continue

                visited_states.append(result)
                visited.add(grid_position)
                count += 1
                
                if debug:
                    self.displayVisited((result.x.astype(int), result.y.astype(int)))
        
        if(count >= self.max_count):
            print("Max Count Exceeded")
        if debug:
            return visited_states
        # If the max count is exceeded, then return None, meaning no path was found
        return None
    
    def actions(self, state:State):
        velocities = [
            max(state.v - 10, -self.robot.v_max),
            state.v,
            min(state.v + 10, self.robot.v_max)
        ]
        steering_angles = [
            state.steering_angle - np.deg2rad(2),
            state.steering_angle,
            state.steering_angle + np.deg2rad(2)
        ]
        action_list = list()
        for angle in steering_angles:
            for v in velocities:
                action_list.append((v, angle))
        return action_list
    
    def results(self, state: State, action):
        x, y, theta, v, omega = self.robot.kinematics(self.dt, action[0], action[1], state.x, state.y, state.theta)
        return State((x,y), theta, v, omega, steering_angle=action[1], parent=state)

    def calculateCost(self, prev_state: State, new_state: State):
        distance_cost = self.heuristic(new_state) 
        # Avoid constantly changing velocities (i.e. going back and forth in a zig zag pattern)
        control_effort_cost = (abs(prev_state.v) - abs(new_state.v))
        # We want to avoid turning, so increase the cost if the wheel velocities are different.
        turning_cost = abs(new_state.omega)
        return distance_cost + 0.1 * turning_cost + control_effort_cost
    
    def heuristic(self, state: State):
        distance = np.sqrt((self.goal.y - state.y)**2 + (self.goal.x - state.x)**2)
        return distance
