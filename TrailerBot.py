from Robot import *
from PoliceCar import * 
from DeliveryBot import *
import pygame
import numpy as np
import Utils
class TrailerBot(Robot):
    def __init__(self, start):
        super().__init__(start)

        self.v = 50
        self.L = 2.8  # m
        self.d = 5.0 * self.m2p  # converting to pixels to make it nice to draw
        self.w = 1.75  # meters

        self.car = PoliceCar(start)
        self.trailer_x = start[0] - self.d
        self.trailer_y = start[1]
        self.trailer_phi = 0
        self.trailer_zeta = 0

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


    def new_kinematics(self, initial_states, trailer_angle, dt, v, omega):
        car_state = self.car.kinematics(initial_states, dt, v, omega)
        car_x, car_y, car_phi, car_zeta = car_state
        phi = trailer_angle + dt * ((v/self.d) * np.sin(car_phi-trailer_angle))
        x = car_x - (self.d * np.cos(phi))
        y = car_y - (self.d * np.sin(car_phi))
        return ((car_x, car_y, car_phi, car_zeta), (x, y, phi))

    def new_step(self, dt):
        initial_car_states = (self.x, self.y, self.theta, self.steering_angle)

        car_state, trailer_state = self.new_kinematics(initial_car_states, self.trailer_phi, dt, self.v, self.omega)
        self.x, self.y, self.theta, self.steering_angle = car_state
        self.trailer_x, self.trailer_y, self.trailer_phi = trailer_state

        self.car_rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.car_rect = self.car_rotated.get_rect(center=(self.x, self.y))
        self.trailer_rotated = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(-self.theta), 1)
        self.trailer_rect = self.trailer_rotated.get_rect(center=(self.trailer_x, self.trailer_y))

    def step(self, dt):
        initial_car_states = (self.x, self.y, self.theta, self.steering_angle)
        new_car_state = self.car_kinematics(initial_car_states, dt, self.v, self.omega)
        self.x, self.y, self.theta, self.steering_angle = new_car_state
        self.trailer_x, self.trailer_y, self.trailer_phi = self.trailer_kinematics(
            (self.trailer_x, self.trailer_y, self.trailer_phi),
            new_car_state,
            self.v,
            dt
        )
        self.car_rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.car_rect = self.car_rotated.get_rect(center=(self.x, self.y))
        self.trailer_rotated = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(-self.theta), 1)
        self.trailer_rect = self.trailer_rotated.get_rect(center=(self.trailer_x, self.trailer_y))

    def car_kinematics(self, initial_states: tuple[float], dt, v, omega):
        init_x, init_y, init_theta, init_zeta = initial_states
        zeta_prime = Utils.clamp(init_zeta + (dt * omega), -np.deg2rad(10), np.deg2rad(10))
        theta_prime = init_theta + dt * (np.tan(zeta_prime) / self.L)
        x_prime = init_x + (dt * (v*np.cos(theta_prime)))
        y_prime = init_y + (dt * (v*np.sin(theta_prime)))
        return x_prime, y_prime, theta_prime, zeta_prime
    
    def trailer_kinematics(self, initial_trailer_state: tuple[float], car_state: tuple[float], v, dt):
        x0, y0, phi0, zeta0 = initial_trailer_state
        car_x, car_y, car_phi, car_zeta = car_state
        # phi = phi0 + dt * ((car_v/self.d) * np.sin(car_phi-phi0))
        # x = car_x - (self.d * np.cos(phi))
        # y = car_y - (self.d * np.sin(car_phi))
        x = x0 + dt * (v * np.cos(car_phi + phi0))
        y = y0 + dt * (v * np.sin(car_phi + phi0))
        car_turn_radius = self.L / np.tan(car_zeta) if car_zeta != 0 else 0
        trailer_turn_radius = car_turn_radius - self.d
        trailer_steering_angle = np.arctan2(self.w, trailer_turn_radius)
        phi = phi0 + dt * (v * np.tan(trailer_steering_angle) / self.d)
        return x, y, phi

    def draw(self, map):
        map.blit(self.car_rotated, self.car_rect)
        map.blit(self.trailer_rotated, self.trailer_rect)
    
    def checkCollision(self, environment: Environment, car_position, car_angle, trailer_position, trailer_angle):
        hitbox_rect = pygame.transform.rotozoom(self.img, np.rad2deg(car_angle), 1).get_rect(center=(car_position[0], car_position[1]))
        trailer_hitbox_rect = pygame.transform.rotozoom(self.trailer_img, np.rad2deg(trailer_angle), 1).get_rect(center=(trailer_position[0], trailer_position[1]))
        # We want to check if the center of the robot is off the screen.
        is_robot_off_screen = car_position[0] < 0 or car_position[0] > environment.width or car_position[1] < 0 or car_position[1] > environment.height
        return environment.checkCollision(hitbox_rect) or environment.checkCollision(trailer_hitbox_rect) or is_robot_off_screen

test_bot = TrailerBot((50,50))
result = test_bot.new_kinematics((50, 50, 0, 0), 0, 1, 100, np.deg2rad(10))
print(result)
