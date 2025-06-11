import pygame
import sys
import threading
import queue

class Point():
    cell_size = -1
    window_width = -1
    window_height = -1
    origin = (-1, -1)
    def __init__(self, x, y):
        self.x =  (self.window_width * ( self.origin[0] / 100 ) ) + x * self.cell_size
        self.y =  (self.window_height * ( self.origin[1] / 100 ) ) - y * self.cell_size
    
    def __str__(self):
        return f"Point({self.x}, {self.y})"

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

class Renderer():
    def __init__(self, window_width, window_height, cell_size = 25, origin = (5, 95), title= '', agents_default_param = {}, agents_init_pos = [], map = []):
        self.window_width = window_width
        self.window_height = window_height
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        # pygame.display.set_caption(title)
        self.cell_size = cell_size

        # Point.cell_size = cell_size
        # Point.window_width = window_width
        # Point.window_height = window_height
        # Point.origin = origin

        self.step = -1
        # self.origin = (10, 80)
        self.origin = origin

        self.agents_default_param = agents_default_param
        self.agents_init_pos = agents_init_pos
        self.map = [['1', '1', '1'], ['0', '1', '1']]

        self.sightRadius = -1
        self.radius = -1

    def set_title(self, title): 
        pygame.display.set_caption(title)

    def set_agent_default_param(self, agents_default_param):
        self.agents_default_param = agents_default_param
    
    def set_agents_init_pos(self, agents_init_pos):
        self.agents_init_pos = agents_init_pos

    def set_map(self, map):
        self.map = map

    def get_agents_init_pos(self):
        return self.agents_init_pos
    
    def get_agents_default_param(self):
        return self.agents_default_param
    
    def get_map(self):
        return self.map
    
    def transform_coordinates(self, x, y):
        x_transformed =  (self.window_width * ( self.origin[0] / 100 ) ) + x * self.cell_size
        y_transformed =  (self.window_height * ( self.origin[1] / 100 ) ) - y * self.cell_size

        return x_transformed, y_transformed
    
    def draw_background(self, color):
        self.screen.fill(color)
    
    def draw_transparent_circle(self, x, y, radius = None, color = (255, 255, 255, 128)):
        COLOR = (255, 255, 255, 128)  # Color with transparency
        if radius is None:
            radius = self.cell_size // 2

        x, y = self.transform_coordinates(x, y)
        surface = pygame.Surface((2*radius, 2*radius), pygame.SRCALPHA)
        pygame.draw.circle(surface, color, (radius, radius), radius)
        self.screen.blit(surface, (x - radius, y - radius))
    
    def draw_text(self, text = "Hello, Pygame!", coord = None, size = None):
        if coord == None:
            coord = (self.window_width // 2, self.window_height // 2)
        else:
            coord = self.transform_coordinates(coord[0], coord[1])
        
        if size is None:
            size = self.cell_size

        font = pygame.font.Font(None, size)
        text = font.render(text, True, (255, 255, 255))
        text_rect = text.get_rect(center= coord)
        self.screen.blit(text, text_rect)
    
    def draw_grid(self):
        color_grid = (175, 175, 175)
        color_axis = (0, 0, 0)

        axis_height = self.cell_size * 0.01
        axis_width = int (self.cell_size * 0.1)
        origin = self.transform_coordinates(0, 0)

        n_vertical_lines = self.window_width // self.cell_size
        n_horizontal_lines = self.window_height // self.cell_size

        for x in range(0, n_vertical_lines):
            curr_point = self.transform_coordinates(x, 0)
            pygame.draw.line(self.screen, color_grid, (curr_point[0], 0), (curr_point[0], self.window_height))

            if x % 5 == 0:
                pygame.draw.line(self.screen, color_axis, self.transform_coordinates(x, - axis_height), self.transform_coordinates(x, axis_height), axis_width)
                self.draw_text(str(x), (x, - axis_height * 2.0))

        pygame.draw.line(self.screen, color_axis, origin, self.transform_coordinates(0, n_horizontal_lines), axis_width)
        pygame.draw.line(self.screen, color_axis, origin, self.transform_coordinates(n_vertical_lines, 0), axis_width)

        for y in range(0, n_horizontal_lines):
            curr_point = self.transform_coordinates(0, y)
            pygame.draw.line(self.screen, color_grid, (0, curr_point[1]), (self.window_width, curr_point[1]))

            if y % 5 == 0 and y != 0:
                pygame.draw.line(self.screen, color_axis, self.transform_coordinates(- axis_height, y), self.transform_coordinates(axis_height, y))
                self.draw_text(str(y), (- axis_height * 2.0, y))
    
    def draw_agent(self, x, y, color = (255, 0, 0), radius = None, id = -1, text = 'mucho texto', sightRadius = None):
        BLACK = (0, 0, 0)

        if radius is None:
            radius = self.cell_size * self.radius
        else:
            radius = self.cell_size * radius

        if sightRadius is None:
            sightRadius = self.cell_size * self.sightRadius
        
        x_transformed, y_transformed = self.transform_coordinates(x, y)

        circle_surface = pygame.Surface((100, 100), pygame.SRCALPHA)

        self.draw_transparent_circle(x, y, radius=sightRadius, color=(255, 255, 255, 128))
        pygame.draw.circle(self.screen, BLACK, (x_transformed, y_transformed), radius + 1)    # margen
        pygame.draw.circle(self.screen, color, (x_transformed, y_transformed), radius)
        self.draw_text(f'{text}: {id}', (x, y), size=self.cell_size // 2)
    
    def update_display(self):
        pygame.display.flip()
    
    def draw_agents(self, agents):
        pass

    def draw_obstacles(self):
        height = len(self.map)

        for y, row in enumerate(self.map):
            y_inverted = height - 1 - y
            for x, cell in enumerate(row):
                if cell == '1':
                    self.draw_obstacle(x, y_inverted)
                    pass

    def draw_obstacle(self, x, y, color = (255, 0, 0)):     # Usa como referencia el punto inferior izquierdo de la celda
        
        y += 1
        x, y = self.transform_coordinates(x, y)

        pygame.draw.rect(self.screen, color, (x, y, self.cell_size, self.cell_size))
    
    def draw_obstacle2(self, point = (-1, -1), color = (255, 0, 0)):
        # Usa como referencia el punto inferior izquierdo de la celda
        y += 1
        x, y = self.transform_coordinates(x, y)

        pygame.draw.rect(self.screen, color, (x, y, self.cell_size, self.cell_size))

    def start(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.draw_background((135, 206, 250)) 
            self.draw_obstacles()
            self.draw_grid()
            self.draw_agent(2.5,3.5, color=(175, 0, 0), text='ID', radius= 0.5)
            self.update_display()
            

        # self.quit()
    
    def quit(self):
        pygame.quit()
        sys.exit()
    

if __name__ == "__main__":
    # # Inicializar Pygame
    pygame.init()

    Renderer(800, 600).start()
