from Robot import *
import pygame
import numpy as np
import Utils
class PoliceCar(Robot):
    def __init__(self, start):
        super().__init__(start)
        # Graphics
        self.img_path = r"DelieveryRobot.png"
        image = pygame.transform.scale(pygame.image.load(self.img_path),(60, 160))
        self.img = pygame.transform.rotozoom(image, -90, 1)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        self.v = 50
        self.L = 3.0  # Meters
        self.max_psi = np.deg2rad(10)
    
    def kinematics(self, initial_states: tuple[float], dt, v, omega):
        """Calculates the kinematics of the PoliceCar

        Args:
            initial_states (tuple[float]): (x, y, phi, psi)
            dt (_type_): time step
            v (_type_): Robot velocity
            omega (_type_): Steering Angle acceleration

        Returns:
            tuple: The new kinematic values as a tuple (x, y, phi, psi)
        """
        init_x, init_y, init_theta, init_psi = initial_states
        psi_prime = Utils.clamp(init_psi + (dt * omega), -self.max_psi, self.max_psi)
        theta_prime = init_theta + (dt * (v * (np.tan(psi_prime) / self.L)))
        x_prime = init_x + (dt * (v*np.cos(theta_prime)))
        y_prime = init_y + (dt * (v*np.sin(theta_prime)))
        return x_prime, y_prime, theta_prime, psi_prime

    def step(self, dt):
        """Step through the kinematics for a specific amount of time. Updates variables automatically

        Args:
            dt (float): Amount of time to calculate kinematics
        """
        initial_states = (self.x, self.y, self.theta, self.omega)
        self.x, self.y, self.theta, self.omega = self.kinematics(initial_states, dt, self.v, self.omega)
