import pygame
import copy
from DelieveryBot import *
from Environment import *
from Search import *

start = (50, 50)
bot = DelieveryBot(start)
screen_dims = (700, 1400)

goal = pygame.Rect((500, 550), (300, 100))
environment = Environment(screen_dims, "RBE550 - Valet", goal)

# Place obstacles
environment.createObstacle((100, 550), (300, 100))
environment.createObstacle((900, 550), (300, 100))
environment.createObstacle((500, 100), (300, 300))

goal_state = State((600, 700), 0, 0.0, 0.0)
search = Search(bot, goal_state, environment)
print("Searching...")
path = search.search(debug=True)

if path is None:
    print("No Path Found...")
    exit()
print("Path Found!")

pygame.init()

running = True
clock = pygame.time.Clock()
path_node = path.pop(0) # First element
last_time = pygame.time.get_ticks()
environment.map.fill((50, 50, 50))
environment.placeAreasOfInterest()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            continue
    if len(path) == 0:
        continue
    dt = clock.tick(60) / 1000.0  # Convert milliseconds to seconds
    # environment.map.fill((50, 50, 50))
    # environment.placeAreasOfInterest()
    environment.map.set_at((int(path_node.x), int(path_node.y)), (255, 255, 255))
    # print(path_node)
    # pygame.draw.rect(environment.map, (255, 0, 255), bot.rect)
    
    # bot.draw(environment.map)
    
    current_time = pygame.time.get_ticks()
    if current_time - last_time >= 10:
        path_node = path.pop(0)
        last_time = current_time

    # pygame.draw.line(
    #     environment.map,
    #     (255, 0, 0),
    #     (bot.x, bot.y),
    #     environment.goal_coord
    # )
    
    
    unit_vector = Utils.calculateUnitVector(bot.theta)
    scale = 30
    robot_direction = (
        bot.x + unit_vector[0]*scale,
        bot.y + unit_vector[1]*scale
    )
    pygame.draw.line(
        environment.map,
        (0, 255, 0),
        (bot.x, bot.y),
        robot_direction,
        width=2
    )
    pygame.display.flip()

pygame.quit()
    
