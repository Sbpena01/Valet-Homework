import pygame
import copy
from DeliveryBot import *
from Environment import *
from DeliveryBotSearch import *

start = (50, 50)
bot = DeliveryBot(start)
screen_dims = (700, 1400)

goal = pygame.Rect((500, 550), (300, 100))
environment = Environment(screen_dims, "RBE550 - Valet", goal)

# Place obstacles
environment.createObstacle((100, 550), (300, 100))
environment.createObstacle((900, 550), (300, 100))
environment.createObstacle((500, 100), (300, 300))

goal_state = State(environment.goal_coord, 0, 0.0, 0.0)
search = DeliveryBotSearch(bot, goal_state, environment)
environment.map.fill((50, 50, 50))
environment.placeAreasOfInterest()

pygame.display.flip()
print("Searching...")
path = search.search(debug=True)
print(len(path))
if path is None:
    print("No Path Found...")
    exit()

pygame.init()

running = True
clock = pygame.time.Clock()
path_node = path.pop(0) # First element
last_time = pygame.time.get_ticks()

count = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            continue
    if len(path) == 0:
        continue
    dt = clock.tick(60) / 1000.0  # Convert milliseconds to seconds
    environment.map.set_at((int(path_node.x), int(path_node.y)), (255, 255, 255))


    current_time = pygame.time.get_ticks()
    if current_time - last_time >= 20:
        # path_node = path.pop(0)
        last_time = current_time
    
    unit_vector = Utils.calculateUnitVector(bot.theta)
    scale = 30
    robot_direction = (
        bot.x + unit_vector[0]*scale,
        bot.y - unit_vector[1]*scale
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
    
