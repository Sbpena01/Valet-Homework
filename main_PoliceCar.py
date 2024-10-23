import pygame
import copy
from PoliceCar import *
from Environment import *
from PoliceCarSearch import *

pygame.init()
running = True
clock = pygame.time.Clock()

start = (50, 50)
bot = PoliceCar(start)
screen_dims = (700, 1400)

goal = pygame.Rect((500, 550), (300, 100))
environment = Environment(screen_dims, "RBE550 - Valet", goal)

# Place obstacles
environment.createObstacle((100, 550), (300, 100))
environment.createObstacle((900, 550), (300, 100))
environment.createObstacle((500, 100), (300, 300))

environment.map.fill((50, 50, 50))
environment.placeAreasOfInterest()
# bot.draw(environment.map)

goal_state = State(environment.goal_coord, 0, 0.0, 0.0)
pygame.draw.circle(environment.map, (255, 0, 255), (goal_state.x, goal_state.y), 4)
search = PoliceCarSearch(bot, goal_state, environment)
print("Searching...")

pygame.display.flip()
path = search.search()
if path is None:
    print("No Path Found...")
    exit()
path_node = path.pop(0) # First element
last_time = pygame.time.get_ticks()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            continue
    if len(path) == 0:
        continue
    dt = clock.tick(60) / 1000.0  # Convert milliseconds to seconds
    environment.map.fill((50, 50, 50))
    environment.placeAreasOfInterest()
    # pygame.draw.rect(environment.map, pygame.Color(255, 0, 255, a=100), bot.rect)
    
    # bot.draw(environment.map)
    environment.drawPath(path.copy())
    current_time = pygame.time.get_ticks()
    if current_time - last_time >= dt:
        path_node = path.pop(0)
        last_time = current_time
    bot.x = path_node.x
    bot.y = path_node.y
    bot.theta = path_node.theta
    # # bot.vl = path_node.vl
    # # bot.vr = path_node.vr
    # bot.omega = np.deg2rad(48)
    # bot.v = 100
    # bot.step(dt)

    pygame.draw.circle(environment.map, (255, 0, 255), (goal_state.x, goal_state.y), 4)
    unit_vector = Utils.calculateUnitVector(bot.theta)
    scale = 30
    robot_direction = (
        bot.x + unit_vector[0]*scale,
        bot.y - unit_vector[1]*scale
    )
    pygame.draw.line(
        environment.map,
        (0, 150, 0),
        (bot.x, bot.y),
        robot_direction,
        width=3
    )
    pygame.draw.circle(
        environment.map,
        (255, 255, 0),
        (bot.x, bot.y),
        3.0
    )
    
    pygame.display.flip()

pygame.quit()
    
