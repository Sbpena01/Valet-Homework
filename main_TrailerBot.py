import pygame
from TrailerBot import *
from Environment import *
from TrailerBotSearch import *

pygame.init()
running = True
clock = pygame.time.Clock()

start = (100, 50)
bot = TrailerBot(start)
screen_dims = (700, 1400)

goal = pygame.Rect((500, 550), (300, 100))
environment = Environment(screen_dims, "RBE550 - Valet", goal)

# Place obstacles
environment.createObstacle((900, 550), (300, 100))
environment.createObstacle((500, 100), (300, 300))

environment.map.fill((50, 50, 50))
environment.placeAreasOfInterest()

goal_x, goal_y = environment.goal_coord
goal_car_state = (goal_x, goal_y, 0, 0)
goal_trailer_state = (goal_x - 5.0 * environment.m2p, goal_y, 0, 0)

goal_state = State(environment.goal_coord, 0, 0, 0, 0)
pygame.draw.circle(environment.map, (255, 0, 255), (goal_state.x, goal_state.y), 4)
search = TrailerBotSearch(bot, goal_state, environment)
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
    dt = clock.tick(30) / 1000.0  # Convert milliseconds to seconds
    environment.map.fill((50, 50, 50))
    environment.placeAreasOfInterest()
    # pygame.draw.rect(environment.map, pygame.Color(255, 0, 255, a=100), bot.rect)
    
    environment.drawTrailerPath(path.copy())
    bot.draw(environment.map)
    current_time = pygame.time.get_ticks()
    if current_time - last_time >= dt*100:
        path_node = path.pop(0)
        last_time = current_time
    bot.x = path_node.car_x
    bot.y = path_node.car_y
    bot.theta = path_node.car_phi
    bot.trailer_x = path_node.trailer_x
    bot.trailer_y = path_node.trailer_y
    bot.trailer_phi = path_node.trailer_phi


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

    unit_vector = Utils.calculateUnitVector(bot.trailer_phi)
    trailer_direction = (
        bot.trailer_x + unit_vector[0]*scale,
        bot.trailer_y - unit_vector[1]*scale
    )
    # pygame.draw.line(
    #     environment.map,
    #     (0, 150, 0),
    #     (bot.trailer.x, bot.trailer.y),
    #     trailer_direction,
    #     width=3
    # )
    pygame.draw.circle(
        environment.map,
        (255, 0, 255),
        (bot.trailer_x, bot.trailer_y),
        3.0
    )
    pygame.draw.line(
        environment.map,
        (0, 0, 0),
        (bot.trailer_x, bot.trailer_y),
        (bot.x, bot.y),
        width=3
    )
    bot.draw(environment.map)
    
    pygame.display.flip()

pygame.quit()
    
