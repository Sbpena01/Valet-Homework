class Robot:
    def __init__(self, start: tuple):
        self.max_v = 20.0  # Max linear velocity
        self.max_w = 10.0  # Max angular velocity
        self.v = 100.0  # Linear velocity
        self.w = 0.0  # Angular velocity
        self.L = 100  # Wheelbase in meters
        self.steering_angle = 0.0   # Steering angle of robot
        self.vl = 0.0  # Velocity of left wheel
        self.vr = 0.0  # Velocity of right wheel
        
        # Odom information
        self.x = start[0]
        self.y = start[1]
        self.theta = 0.0  # radians
    
    def step(self):
        """Calculates the current location of the robot via robot kinematics.
        """
        raise NotImplementedError
    
    def draw(self, map):
        """Draws the robot on the screen
        """
        raise NotImplementedError
