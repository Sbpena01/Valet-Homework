from Search import *
from PoliceCar import *

class PoliceCarSearch(Search):
    def __init__(self, robot: PoliceCar, goal, environment):
        super().__init__(robot, goal, environment)
        self.robot = robot
        self.dt = 0.1

    def search(self, debug=False):
        self.search_queue.put((0, self.start))
        count = 0
        visited = set()
        visited_states = list()
        while not self.search_queue.empty() and count < self.max_count:
            _, current_state = self.search_queue.get()
            print()
            if self.checkGoal(current_state):
                print("Found a path!")
                if debug:
                    return visited_states
                return self.getPath(current_state)
            actions = self.actions()
            for action in actions:
                result = self.results(current_state, action)
                grid_position = (
                    int(result.x * 10),
                    int(result.y * 10)
                )

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
                print(result)
                if debug:
                    self.displayVisited((result.x.astype(int), result.y.astype(int)))
                    # print(result.steering_angle, result.v)
        
        if(count >= self.max_count):
            print("Max Count Exceeded")
        if debug:
            return visited_states
        return None
    
    def actions(self):
        omegas = [  # Angular velocity of steering angle
            np.deg2rad(-30),
            np.deg2rad(-20),
            np.deg2rad(-10),
            np.deg2rad(0),
            np.deg2rad(20),
            np.deg2rad(10),
            np.deg2rad(30),
        ]
        velocities = [
            -self.robot.v,
            self.robot.v,
        ]
        action_list = list()
        for omega in omegas:
            for v in velocities:
                action_list.append((v, omega))
        return action_list
    
    def results(self, state: State, action):
        initial_state = (
            state.x,
            state.y,
            state.theta,
            state.steering_angle
        )
        v = action[0]
        omega = action[1]
        x, y, theta, steering_angle = self.robot.kinematics(initial_state, self.dt, v, omega)
        return State((x,y), theta, v, omega, steering_angle, state)
    
    def calculateCost(self, prev_state: State, new_state: State):
        distance_cost = self.heuristic(new_state)
        control_effort_cost = abs(new_state.v - prev_state.v)
        # control_effort_cost = 0
        return distance_cost + control_effort_cost
    
