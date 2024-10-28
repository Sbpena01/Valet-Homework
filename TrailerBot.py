from Robot import *
from PoliceCar import * 
from DeliveryBot import *
import pygame
import numpy as np
import Utils
class TrailerBot(Robot):
    def __init__(self, start):
        super().__init__(start)

        self.v = 50  # Truck velocity
        self.L = 2.8  # m  Wheelbase
        self.d = 5.0 * self.m2p  # Trailer axle length; converting to pixels to make it nice to draw
        self.w = 1.75  # Trailer axle width

        self.car = PoliceCar(start)  # Truck pulling the trailer
        # Trailer kinematic information
        self.trailer_x = start[0] - self.d
        self.trailer_y = start[1]
        self.trailer_phi = 0
        self.trailer_psi = 0

        # Graphics
        self.img_path = r"DelieveryRobot.png"
        self.trailer_img_path = r"DelieveryRobot.png"
        image = pygame.transform.scale(pygame.image.load(self.img_path), (60, 100))
        trailer_image = pygame.transform.scale(pygame.image.load(self.trailer_img_path),(50, 100))
        self.img = pygame.transform.rotozoom(image, -90, 1)
        self.trailer_img = pygame.transform.rotozoom(trailer_image, -90, 1)
        self.car_rotated = self.img
        self.car_rect = self.car_rotated.get_rect(center=(self.x, self.y))
        self.trailer_rotated = self.trailer_img
        self.trailer_rect = self.trailer_rotated.get_rect(center=(self.trailer_x, self.trailer_y))


    def kinematics(self, initial_states, trailer_angle, dt, v, omega):
        """Calculates the kinematics of both the truck and trailer

        Args:
            initial_states (tuple): The initial truck state
            trailer_angle (float): Initial trailer angle
            dt (float): Time step
            v (float): Truck velocity
            omega (float): Truck steering angle velocity

        Returns:
            tuple: Both the kinematic variables for the truck and trailer.
        """
        car_state = self.car.kinematics(initial_states, dt, v, omega)
        car_x, car_y, car_phi, car_psi = car_state
        phi = trailer_angle + dt * ((v/self.d) * np.sin(car_phi-trailer_angle))
        x = car_x - (self.d * np.cos(phi))
        y = car_y - (self.d * np.sin(car_phi))
        return ((car_x, car_y, car_phi, car_psi), (x, y, phi))

    def step(self, dt):
        """Step through the kinematics for a specific amount of time. Updates variables automatically

        Args:
            dt (float): Amount of time to calculate kinematics
        """
        initial_car_states = (self.x, self.y, self.theta, self.steering_angle)

        car_state, trailer_state = self.kinematics(initial_car_states, self.trailer_phi, dt, self.v, self.omega)
        self.x, self.y, self.theta, self.steering_angle = car_state
        self.trailer_x, self.trailer_y, self.trailer_phi = trailer_state

        self.car_rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.car_rect = self.car_rotated.get_rect(center=(self.x, self.y))
        self.trailer_rotated = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(-self.theta), 1)
        self.trailer_rect = self.trailer_rotated.get_rect(center=(self.trailer_x, self.trailer_y))

    def car_kinematics(self, initial_states: tuple[float], dt, v, omega):
        """Calculates the kinematics of the truck.

        Args:
            initial_states (tuple[float]): Initial kinematic state of the truck
            dt (float): Time step
            v (float): Robot velocity
            omega (float): Steering angle velocity

        Returns:
            tuple: New kinematic state of the truck
        """
        init_x, init_y, init_theta, init_psi = initial_states
        psi_prime = Utils.clamp(init_psi + (dt * omega), -np.deg2rad(10), np.deg2rad(10))
        theta_prime = init_theta + dt * (np.tan(psi_prime) / self.L)
        x_prime = init_x + (dt * (v*np.cos(theta_prime)))
        y_prime = init_y + (dt * (v*np.sin(theta_prime)))
        return x_prime, y_prime, theta_prime, psi_prime

    def draw(self, map):
        """Draws the truck and trailer in the environment using pygame.

        Args:
            map (Environment): The environment to draw on.
        """
        self.car_rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.car_rect = self.car_rotated.get_rect(center=(self.x, self.y))
        self.trailer_rotated = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(-self.theta), 1)
        self.trailer_rect = self.trailer_rotated.get_rect(center=(self.trailer_x, self.trailer_y))
        
        map.blit(self.car_rotated, self.car_rect)
        map.blit(self.trailer_rotated, self.trailer_rect)
    
    def checkCollision(self, environment: Environment, car_position, car_angle, trailer_position, trailer_angle):
        """Checks if either the truck and/or trailer collided with an obstacle

        Args:
            environment (Environment): Environment to check collision within
            car_position (tuple): The (x,y) position of the truck
            car_angle (float): The angle of the truck in radians
            trailer_position (tuple): The (x,y) position of the trailer
            trailer_angle (float): The angle of the trailer

        Returns:
            bool: True if a collision has occured. False otherwise.
        """
        hitbox_rect = pygame.transform.rotozoom(self.img, np.rad2deg(car_angle), 1).get_rect(center=(car_position[0], car_position[1]))
        trailer_hitbox_rect = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(trailer_angle), 1).get_rect(center=(trailer_position[0], trailer_position[1]))
        # We want to check if the center of the robot is off the screen.
        is_robot_off_screen = car_position[0] < 0 or car_position[0] > environment.width or car_position[1] < 0 or car_position[1] > environment.height
        return environment.checkCollision(hitbox_rect) or environment.checkCollision(trailer_hitbox_rect) or is_robot_off_screen
