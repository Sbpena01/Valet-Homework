from Robot import Robot
from Environment import *
import numpy as np
from queue import PriorityQueue

class State:
    def __init__(self, position: tuple, angle, v, omega, steering_angle = 0.0, parent=None):
        # Contains all the kinematic information the robot may need for each 'node' in the planning algorithm.
        
        self.x = position[0]
        self.y = position[1]
        self.theta = angle
        self.v = v
        self.omega = omega
        self.steering_angle = steering_angle
        self.parent = parent  # Parent state to get to this state. Defaults to None (i.e. starting state)
        self.cost = 100000  # Cost of the state. Defaults to a high cost for the starting node.
        
    def __str__(self):
        return f"Position: {self.x}, {self.y} V: {self.v} Steering Angle: {np.rad2deg(self.steering_angle)}"
    
    def __eq__(self, value):
        if not isinstance(value, State):
            return NotImplementedError
        return self.cost == value.cost

class Search:
    def __init__(self, robot: Robot, goal: State, environment: Environment):
        self.robot = robot
        self.map = environment
        self.search_queue = PriorityQueue()
        self.start = State((robot.x, robot.y), robot.theta, robot.v, robot.steering_angle)
        self.goal = goal
        self.max_count = 6000  # Maximum number of loops before search algorithm gives up.
        self.dt = 0.75  # Time step for kinematics
        self.grid_size = 0.1  # Divides the environment into grid cells.
        
    def search(self, debug=False):
        """The main kinematic planner algorithm

        Args:
            debug (bool, optional): _description_. Defaults to False.

        Raises:
            NotImplementedError: Each robot has a different algorithm
        """
        raise NotImplementedError
    
    def to_grid(self, position):
        # Convert pixel coordinates to grid coordinates
        return (int(position[0] // self.grid_size), int(position[1] // self.grid_size))

    def displayVisited(self, pixel):
        """Debugging tool that highlights a pixel that is visited by the planner

        Args:
            pixel (tuple): (x,y) pixel position to highlight
        """
        pygame.Surface.set_at(self.map.map, pixel, (255, 255, 255))
        pygame.display.flip()

    def getPath(self, state: State):
        """Gets the path from the starting state to the given state.

        Args:
            state (State): State to get the path of

        Returns:
            list: Path from the starting state to the input state.
        """
        path = list()
        while state.parent is not None:
            path.append(state)
            state = state.parent
        path.reverse()
        return path
    
    def checkGoal(self, state: State):
        """Checks if the given state is the goal
        """
        return self.heuristic(state) <= 2
    
    def actions(self):
        """Creates a list of possible actions the robot can take

        Returns:
            list: List of actions (control inputs) the robot can use
        """
        raise NotImplementedError
    
    def results(self, state: State, action):
        """Returns a new state containing the information regarding the new kinematic state after
        inputting the control inputs

        Args:
            state (State): The initial robot state
            action (tuple): The control inputs
        """
        raise NotImplementedError

    def calculateCost(self, prev_state: State, new_state: State):
        """Calculates the cost of the state

        Args:
            prev_state (State): Parent state
            new_state (State): Child state
        """
        raise NotImplementedError
    
    def heuristic(self, state: State):
        """Calculates and returns the euclidean distance from the given state to the goal.
        """
        distance = np.sqrt((self.goal.y - state.y)**2 + (self.goal.x - state.x)**2)
        return distance
    
