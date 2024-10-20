import pygame
import sys
from planning import *

# Colors
WHITE = (255, 255, 255)
LIGHT_GRAY = (200, 200, 200)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class Simulator():
    SCALE = 3
    SPEED = 2
    NUM_VISIBLE_WAYPOINTS = math.inf
    GRID_WIDTH, GRID_HEIGHT = 2000//SCALE, 2000//SCALE
    WINDOW_WIDTH, WINDOW_HEIGHT = GRID_WIDTH, GRID_HEIGHT  # Adjusted to fit the entire grid
    OBSTACLE_WIDTH = 200
    OBSTACLE_LENGTH = 200
    WAYPOINT_RADIUS = 20
    FPS = 100

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((Simulator.WINDOW_WIDTH, Simulator.WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()

    def draw_grid(self):
        for x in range(0, Simulator.GRID_WIDTH, Simulator.GRID_WIDTH//10):
            pygame.draw.line(self.screen, LIGHT_GRAY, (x, 0), (x, Simulator.GRID_HEIGHT))

        for y in range(0, Simulator.GRID_HEIGHT, Simulator.GRID_WIDTH//10):
            pygame.draw.line(self.screen, LIGHT_GRAY, (0, y), (Simulator.WINDOW_WIDTH, y))

    def draw_obstacle(self):
        rect = pygame.rect.Rect((2000 - Simulator.OBSTACLE_WIDTH)//Simulator.SCALE, 0//Simulator.SCALE, Simulator.OBSTACLE_WIDTH//Simulator.SCALE, Simulator.OBSTACLE_LENGTH//Simulator.SCALE)
        pygame.draw.rect(self.screen, BLACK, rect)

    def set_waypoints(self, waypoints: WaypointSequence):
        self.waypoints = waypoints

    def draw_waypoints(self):
        for ind, wp in enumerate(self.waypoints.waypoints):
            if (ind == Simulator.NUM_VISIBLE_WAYPOINTS):
                break
            pygame.draw.circle(self.screen, BLUE, (wp.x//Simulator.SCALE, Simulator.GRID_HEIGHT - wp.y//Simulator.SCALE), Simulator.WAYPOINT_RADIUS)

            if ind:
                pygame.draw.line(self.screen, BLUE, 
                                 (self.waypoints.waypoints[ind].x//Simulator.SCALE, Simulator.GRID_HEIGHT - self.waypoints.waypoints[ind].y//Simulator.SCALE), 
                                 (self.waypoints.waypoints[ind - 1].x//Simulator.SCALE, Simulator.GRID_HEIGHT - self.waypoints.waypoints[ind - 1].y//Simulator.SCALE), 
                                 width=3)

    def simulate(self):
        running = True
        while running:
            self.clock.tick(Simulator.FPS)

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Clear the screen
            self.screen.fill(WHITE)

            self.draw_grid()
            self.draw_obstacle()

            self.draw_waypoints()

            # Update the display
            pygame.display.flip()

        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    sim = Simulator()
    sim.set_waypoints(SnakeWaypointSequence())
    sim.simulate()
