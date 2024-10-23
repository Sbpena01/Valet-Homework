from DelieveryBot import *
from PoliceCar import *
from Environment import *
import numpy as np
from queue import PriorityQueue

class State:
    def __init__(self, position: tuple, angle, v, omega, steering_angle = 0.0, parent=None):
        self.x = position[0]
        self.y = position[1]
        self.theta = angle
        self.v = v
        self.omega = omega
        self.steering_angle = steering_angle
        self.parent = parent
        self.cost = 100000
        
    def __str__(self):
        return f"Position: {self.x}, {self.y} V: {self.v} Steering Angle: {np.rad2deg(self.steering_angle)}"
    
    def __eq__(self, value):
        if not isinstance(value, State):
            return NotImplementedError
        return self.cost == value.cost

class Search:
    def __init__(self, robot: DelieveryBot, goal: State, environment: Environment):
        self.robot = robot
        self.map = environment
        self.search_queue = PriorityQueue()
        self.start = State((robot.x, robot.y), robot.theta, robot.v, robot.steering_angle)
        self.goal = goal
        self.max_count = 6000
        self.dt = 0.75
        self.grid_size = 0.1  # Pixels
        
    def search(self, debug=False):
        self.search_queue.put((0, self.start))
        count = 0
        visited = set()
        visited_states = list()
        while not self.search_queue.empty() and count < self.max_count:
            _, current_state = self.search_queue.get()
            if self.checkGoal(current_state):
                print("Found a path!")
                if debug:
                    return visited_states
                return self.getPath(current_state)
            if len(visited_states) > 200:
                print(count)
                breakpoint = True
            count += 1
            actions = self.actions()
            for action in actions:
                if action[1] > 0:
                    breakpoint = True
                result = self.results(current_state, action)
                if result is None:
                    continue
                grid_position = self.to_grid((result.x, result.y))

                if self.robot.checkCollision(self.map, (result.x, result.y), result.theta):
                    continue
                if grid_position in visited:
                    continue

                result.cost = self.calculateCost(current_state, result)
                try:
                    self.search_queue.put((result.cost, result))
                except TypeError as e:
                    print("Two nodes with the same cost.")
                    continue

                visited_states.append(result)
                visited.add(grid_position)
                if debug:
                    self.displayVisited((result.x.astype(int), result.y.astype(int)))
                    # print(result.x, result.y, result.cost)
        
        if(count >= self.max_count):
            print("Max Count Exceeded")
        if debug:
            return visited_states
        return None
    
    def to_grid(self, position):
        # Convert pixel coordinates to grid coordinates
        return (int(position[0] // self.grid_size), int(position[1] // self.grid_size))

    def displayVisited(self, pixel):
        pygame.Surface.set_at(self.map.map, pixel, (255, 255, 255))
        pygame.display.flip()

    def getPath(self, state: State):
        path = list()
        while state.parent is not None:
            path.append(state)
            state = state.parent
        path.reverse()
        return path
    
    def checkGoal(self, state: State):
        # return self.heuristic(state) <= self.grid_size
        return self.heuristic(state) <= 2
    
    def actions(self):
        steering_angles = [
            np.deg2rad(-4),
            np.deg2rad(-2),
            np.deg2rad(0),
            np.deg2rad(2),
            np.deg2rad(4),
        ]
        velocities = [
            -self.robot.v,
            # -self.robot.v/2,
            # -self.robot.v/4,
            # self.robot.v/4,
            # self.robot.v/2,
            self.robot.v,
        ]
        action_list = list()
        for angle in steering_angles:
            for v in velocities:
                action_list.append((v, angle))
        return action_list
    
    def results(self, state: State, action):
                                                     # dt       v         steering    
        x, y, theta, v, omega = self.robot.kinematics(self.dt, action[0], action[1], state.x, state.y, state.theta)
        return State((x,y), theta, v, omega, steering_angle=action[1], parent=state)

    # def calculateCost(self, prev_state: State, new_state: State):
    #     # We want to avoid turning, so increase the cost if the wheel velocities are different.
    #     distance_cost = self.heuristic(new_state)
    #     # control_effort_cost = abs(new_state.v - prev_state.v) + abs(new_state.theta - prev_state.theta)
    #     # control_effort_cost = 0
    #     return distance_cost
    
    def heuristic(self, state: State):
        distance = np.sqrt((self.goal.y - state.y)**2 + (self.goal.x - state.x)**2)
        return distance
    
# test_state = State((327.8852215982061, 279.2285151716902), 1.3710944427754277, 100, 0.35265396141693, -0.17453292519943295)
# test_bot = PoliceCar((50, 50))
# test_environment = Environment((1000, 1000), 'test', None)
# test_search = Search(test_bot, test_state, test_environment)

# actions = test_search.actions()
# print(len(actions))
# for action in actions:
#     print(action, test_search.results(test_state, action))