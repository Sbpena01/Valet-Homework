import pygame
from Robot import *
# from DelieveryBot import *

class Environment:
    def __init__(self, dims: tuple, name: str, goal: pygame.rect):
        self.height = dims[0]
        self.width = dims[1]
        
        pygame.display.set_caption(name)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.obstacles = []
        self.goal_box = goal
        self.goal_coord = goal.center

        self.m2p = 10  # 1 meter is 10 pixels
        
    def placeAreasOfInterest(self):
        self.placeGoal()
        self.placeObstacles()
    
    def placeGoal(self):
        pygame.draw.rect(self.map, (100, 255, 100), self.goal_box)
    
    def createObstacle(self, origin:tuple, size:tuple):
        obstacle = pygame.Rect(origin, size)
        
        self.obstacles.append(obstacle)
    
    def placeObstacles(self):
        for obstacle in self.obstacles:
            pygame.draw.rect(self.map, (255, 0, 0), obstacle)
    
    def checkCollision(self, rectangle: pygame.Rect):
        return rectangle.collidelist(self.obstacles) != -1
    
    def drawTrailerPath(self, path):
        if len(path) == 0:
            return None
        last = path.pop()
        while len(path) != 0:
            next = path.pop()
            pygame.draw.line(self.map, (255, 0, 255), (int(last.car_x), int(last.car_y)), (int(next.car_x), int(next.car_y)), width=2)
            pygame.draw.line(self.map, (255, 255, 0), (int(last.trailer_x), int(last.trailer_y)), (int(next.trailer_x), int(next.trailer_y)), width=2)
            last = next

    def drawPath(self, path):
        if len(path) == 0:
            return None
        last = path.pop()
        while len(path) != 0:
            next = path.pop()
            pygame.draw.line(self.map, (0, 0, 0), (int(last.x), int(last.y)), (int(next.x), int(next.y)), width=2)
            last = next
    
    def highlightPixel(self, pixel: tuple):
        self.map.set_at((pixel), (0, 0, 255))
                
    
    
        
