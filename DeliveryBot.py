import numpy as np
import pygame
import Environment
from Robot import Robot
import Utils
from Environment import *

class DeliveryBot(Robot):
    def __init__(self, start, width = 1.5):
        super().__init__(start)
        
        # Graphics
        self.img_path = r"DelieveryRobot.png"
        image = pygame.transform.scale(
            pygame.image.load(self.img_path),
            (60, 160)
        )
        self.img = pygame.transform.rotozoom(image, -90, 1)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.R = 1  # Distance between wheels along axle in meters
        self.L = width # Wheelbase in meters
        self.v = 10  # Velocity of the bot
        self.v_max = 20  # Maximum velocity of the bot, cannot exceed this value.
    
    def checkCollision(self, environment: Environment, position, angle):
        """Checks if the robot will collide with an obstacle in the environment at the desired position and angle.

        Args:
            environment (Environment): The environment to check collision in
            position (tuple): The pixel coordinates describing the position of the robot
            angle (float): The orientation of the robot in radians

        Returns:
            bool: True if there is a collision, False if no collision
        """
        hitbox_rect = pygame.transform.rotozoom(self.img, np.rad2deg(angle), 1).get_rect(center=(position[0], position[1]))
        # We also want to check if the center of the robot is off the screen.
        is_robot_off_screen = position[0] < 0 or position[0] > environment.width or position[1] < 0 or position[1] > environment.height
        return environment.checkCollision(hitbox_rect) or is_robot_off_screen
    
    def step(self, dt):
        """Step through the kinematics for a specific amount of time. Updates variables automatically

        Args:
            dt (float): Amount of time to calculate kinematics
        """
        vl, vr = self.controls(self.v, self.steering_angle)
        self.x, self.y, self.theta, self.v, self.steering_angle = self.odom(dt, vl, vr, self.x, self.y, self.theta)
        
        self.rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def kinematics(self, dt, v, steering_angle, x, y, theta):
        """Calculates kinematics of the robot using control inputs v and steering_angle

        Args:
            dt (float): Change of time
            v (float): Robot velocity
            steering_angle (float): Robot steering angle in radians
            x (float): Initial x position
            y (float): Initial y position
            theta (float): Initial angle

        Returns:
            tuple: New kinematic state (position, orientation)
        """
        vl, vr = self.controls(v, steering_angle)
        return self.odom(dt, vl, vr, x, y, theta)
    
    def odom(self, dt: float, vl: float, vr: float, x, y, theta):
        """Uses wheel speeds to calculate new kinematic state (position and orientation)

        Args:
            dt (float): Time step
            vl (float): Left wheel speed
            vr (float): Right wheel speed
            x (float): Initial x position
            y (float ): Initial y position
            theta (float): Initial Angle

        Returns:
            tuple: New kinematic state for the robot
        """
        v = (vl + vr) / 2
        omega = (vl - vr) / self.L
        new_theta = theta + omega * dt
        new_x = x + v * np.cos(new_theta) * dt  # Use new_theta for x
        new_y = y + v * np.sin(new_theta) * dt  # Use new_theta for y
        return new_x, new_y, new_theta, v, omega
    
    def controls(self, v, steering_angle):
        """Uses the control inputs to calculate the wheel speeds

        Args:
            v (float): Robot velocity
            steering_angle (float): Robot steering angle in radians

        Returns:
            tuple: The left and right wheel speeds to achieve the desired velocity and steering angle
        """
        if steering_angle == 0:
            omega = 0
        else:
            R = self.L / (2 * np.tan(steering_angle))
            omega = v / R
        vl = v - (self.L*omega / 2)
        vr = v + (self.L*omega / 2)
        return vl, vr
