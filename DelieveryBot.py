import numpy as np
import pygame
import Environment
from Robot import Robot
import Utils
from Environment import *

class DelieveryBot(Robot):
    def __init__(self, start):
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
    
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def checkCollision(self, environment: Environment, position, angle):
        hitbox_rect = pygame.transform.rotozoom(self.img, np.rad2deg(angle), 1).get_rect(center=(position[0], position[1]))
        # We want to check if the center of the robot is off the screen.
        is_robot_off_screen = position[0] < 0 or position[0] > environment.width or position[1] < 0 or position[1] > environment.height
        return environment.checkCollision(hitbox_rect) or is_robot_off_screen
    
    def step(self, dt):
        vl, vr = self.controls(self.v, self.steering_angle)
        self.x, self.y, self.theta, self.v, self.steering_angle = self.odom(dt, vl, vr, self.x, self.y, self.theta)
        
        self.rotated = pygame.transform.rotozoom(self.img, np.rad2deg(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        
    def goTo(self, pose):
        robot_pose = np.array([self.x, self.y])
        reference_pose = np.array([pose[0], pose[1]])
        pose_error = reference_pose - robot_pose

        robot_angle_vector = Utils.calculateUnitVector(self.theta)
        reference_angle = Utils.calculateAngle(robot_angle_vector, pose_error)
        
        if np.abs(reference_angle) > np.deg2rad(1):
            vl_unclamped = 5 * reference_angle
            vr_unclamped = -5 * reference_angle
        else:
            vl_unclamped = (10 * np.linalg.norm(pose_error)) + 5 * reference_angle
            vr_unclamped = (10 * np.linalg.norm(pose_error)) - 5 * reference_angle
        
        self.vl = Utils.clamp(vl_unclamped, -self.max_v, self.max_v)
        self.vr = Utils.clamp(vr_unclamped, -self.max_v, self.max_v)
        
    def kinematics(self, dt, v, steering_angle, x, y, theta):
        vl, vr = self.controls(v, steering_angle)
        return self.odom(dt, vl, vr, x, y, theta)
    
    def odom(self, dt: float, vl: float, vr: float, x, y, theta):
        v = (vl + vr) / 2
        omega = (vl - vr) / self.L
        new_theta = theta + omega * dt
        new_x = x + v * np.cos(new_theta) * dt  # Use new_theta for x
        new_y = y + v * np.sin(new_theta) * dt  # Use new_theta for y
        return new_x, new_y, new_theta, v, omega
    
    def controls(self, v, steering_angle):
        # Calculates and sets vl and vr
        if steering_angle == 0:
            omega = 0
        else:
            R = self.L / (2 * np.tan(steering_angle))
            omega = v / R
        vl = v - (self.L*omega / 2)
        vr = v + (self.L*omega / 2)
        return vl, vr
