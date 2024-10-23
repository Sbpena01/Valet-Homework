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
        self.L = 2.8  # Meters
    
    def kinematics(self, initial_states: tuple[float], dt, v, omega):
        init_x, init_y, init_theta, init_zeta = initial_states
        zeta_prime = Utils.clamp(init_zeta + (dt * omega), np.deg2rad(-20), np.deg2rad(20))
        theta_prime = init_theta + (dt * (v * (np.tan(zeta_prime) / self.L)))
        x_prime = init_x + (dt * (v*np.cos(theta_prime)))
        y_prime = init_y + (dt * (v*np.sin(theta_prime)))
        return x_prime, y_prime, theta_prime, zeta_prime

    def step(self, dt):
        initial_states = (self.x, self.y, self.theta, self.omega)
        self.x, self.y, self.theta, self.omega = self.kinematics(initial_states, dt, self.v, self.omega)
        # self.rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        # self.rect = self.rotated.get_rect(center=(self.x, self.y))