from Search import *
from TrailerBot import *
from Environment import *
class TrailerState:
    def __init__(self, car_state: tuple, trailer_state: tuple, v=0, omega = 0, zeta=0, parent=None):
        self.car_state = car_state  # (x, y, phi, zeta)
        self.car_x = car_state[0]
        self.car_y = car_state[1]
        self.car_phi = car_state[2]
        self.car_zeta = car_state[3]

        self.trailer_x = trailer_state[0]
        self.trailer_y = trailer_state[1]
        self.trailer_phi = trailer_state[2]
        self.trailer_state = trailer_state  # (x, y, phi)

        self.v = v
        self.omega = omega
        self.zeta = zeta
        self.parent = parent
        self.cost = 100000


class TrailerBotSearch(Search):
    def __init__(self, robot: TrailerBot, goal, environment):
        super().__init__(robot, goal, environment)
        self.start = TrailerState(
            (robot.x, robot.y, robot.theta, robot.steering_angle),
            (robot.trailer_x, robot.trailer_y, robot.trailer_phi)
        )
        self.robot = robot
        self.dt = 0.1

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
            actions = self.actions()
            for action in actions:
                result = self.results(current_state, action)
                grid_position = (
                    int(result.car_x * 10),
                    int(result.car_y * 10)
                )

                if self.robot.checkCollision(self.map, (result.car_x, result.car_y), result.car_phi,
                                             (result.trailer_x, result.trailer_y), result.trailer_phi):
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
                    self.displayVisited((result.car_x.astype(int), result.car_y.astype(int)))
        
        if(count >= self.max_count):
            print("Max Count Exceeded")
        if debug:
            return visited_states
        return None
    
    def actions(self):
        omegas = [  # Angular velocity of zeta
            np.deg2rad(-30),
            np.deg2rad(-20),
            np.deg2rad(-10),
            np.deg2rad(0),
            np.deg2rad(10),
            np.deg2rad(20),
            np.deg2rad(30),
        ]
        velocities = [
            -self.robot.v,
            self.robot.v
        ]
        action_list = list()
        for omega in omegas:
            for v in velocities:
                action_list.append((v, omega))
        return action_list
    
    def results(self, state: TrailerState, action):
        v = action[0]
        omega = action[1]
        new_car_state, new_trailer_state = self.robot.new_kinematics(state.car_state, state.trailer_phi, self.dt, v, omega)
        return TrailerState(new_car_state, new_trailer_state, v=v, omega=omega, parent=state)
    
    def calculateCost(self, prev_state: TrailerState, new_state: TrailerState):
        distance_cost = self.heuristic(new_state)
        control_effort_cost = abs(new_state.v - prev_state.v)
        # trailer_cost = abs(new_state.trailer_x - prev_state.trailer_x) + abs(new_state.trailer_y - prev_state.trailer_y)
        return distance_cost + control_effort_cost 
    
    def heuristic(self, state: TrailerState):
        distance = np.sqrt((self.goal.y - state.car_state[1])**2 + (self.goal.x - state.car_state[0])**2)
        return distance
    

