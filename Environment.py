import pygame
from DelieveryBot import *

class Environment:
    def __init__(self, dims: tuple, name: str, goal: pygame.rect):
        self.height = dims[0]
        self.width = dims[1]
        
        pygame.display.set_caption(name)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.obstacles = []
        self.goal_box = goal
        self.goal_coord = goal.center
        
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
    
    def drawPath(self, path: list):
        if len(path) == 0:
            return None
        last = path.pop()
        while len(path) != 0:
            next = path.pop()
            pygame.draw.line(self.map, (0, 0, 0), (int(last.x), int(last.y)), (int(next.x), int(next.y)), width=2)
            last = next
    
    def highlightPixel(self, pixel: tuple):
        self.map.set_at((pixel), (0, 0, 255))
    
    # def isPixelFree(self, pixel:tuple):
    #     for obstacle in self.obstacles:
    #         if obstacle.collidepoint(pixel):
    #             return True
    #     return False
    
    # def getNeighbors(self, pixel:tuple):
    #     neighbors = [
    #         (pixel[0] + 1, pixel[1]),
    #         (pixel[0] - 1, pixel[1]),
    #         (pixel[0], pixel[1] + 1),
    #         (pixel[0], pixel[1] - 1),
    #         (pixel[0] + 1, pixel[1] + 1),
    #         (pixel[0] + 1, pixel[1] - 1),
    #         (pixel[0] - 1, pixel[1] + 1),
    #         (pixel[0] - 1, pixel[1] - 1),
    #     ]
    #     free_neighbors = []
    #     for neighbor in neighbors:
    #         if self.isPixelFree(neighbor):
    #             free_neighbors.append(neighbor)
    #     return free_neighbors
                
    
    
        
