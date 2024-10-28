import pygame
import numpy as np
class Robot:
    def __init__(self, start: tuple):
        self.v = 20.0  # Linear velocity
        self.omega = 0.0  # Angular velocity
        self.steering_angle = 0.0   # Steering angle of robot
        self.m2p = 30

        # Mainly used for Delievery Bot
        self.vl = 0.0  # Velocity of left wheel
        self.vr = 0.0  # Velocity of right wheel
        
        # Odom information
        self.x = start[0]
        self.y = start[1]
        self.theta = 0.0  # radians

    def draw(self, map):
        """Draws the robot on the environment map.

        Args:
            map (Environment): The environment to draw the robot on.
        """
        self.rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        map.blit(self.rotated, self.rect)

    def checkCollision(self, environment, position, angle):
        """Checks if the robot has collided with an obstacle in the environment

        Args:
            environment (Environment): The environment to check collisions within
            position (tuple): The (x,y) position of the robot
            angle (float): The angle of the robot in radians

        Returns:
            bool: True a collison has occured. False otherwise
        """
        hitbox_rect = pygame.transform.rotozoom(self.img, np.rad2deg(angle), 1).get_rect(center=(position[0], position[1]))
        # We want to check if the center of the robot is off the screen.
        is_robot_off_screen = position[0] < 0 or position[0] > environment.width or position[1] < 0 or position[1] > environment.height
        return environment.checkCollision(hitbox_rect) or is_robot_off_screen

    def step(self):
        """Calculates the current location of the robot via robot kinematics.
        """
        raise NotImplementedError
