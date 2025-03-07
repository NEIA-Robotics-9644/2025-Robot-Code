import pathfinder
import pygame

'''
FILE FORMAT
the first line contains the width and height, seperated by a comma
the next lines are the cell values, in a flattened 2d array
'''
        

def save_cells(cells, filename):
    with open(filename, 'w') as f:
        f.write(f"{resolution_width},{resolution_height}\n")
        for i in range(resolution_height):
            for j in range(resolution_width):
                value = cells[i][j]
                if value:
                    f.write('1\n')
                else:
                    f.write('0\n')

import os
                    
def load_cells(cells, filename):


    # return [[True for i in range(resolution_width)] for j in range(resolution_height)]

    with open(filename, 'r') as f:
        if f is None:
            print("File is none")
        lines = f.readlines()

        width_str, height_str = lines[0].split(',')
        width = int(width_str)
        height = int(height_str)
        

        for i in range(len(cells)):
            for j in range(len(cells[0])):
                cells[i][j] = lines[i * width + j + 1] == "1\n"
                print(lines[i * width + j + 1])

        return cells

angle_size_degs = 150

height_inches = 150

resolution_width = 50 

resolution_height = 50 



cells = [[False for i in range(resolution_width)] for j in range(resolution_height)]

# UI

def main():
    
    load_cells(cells, "cells.txt")
    
    pygame.init()

    screen_width = 800
    screen_height = 600

    graph_width = 500 
    graph_height = 500

    vert_label_number = 10
    horiz_label_number = 10
    
    screen = pygame.display.set_mode((screen_width, screen_height))

    running = True

    mouse_fill_mode = False

    mouse_down = False

    update_path = True

    path = []

    global start_location, end_location

    start_location = (5, 5)
    end_location = (45, 45)

    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                if event.key == pygame.K_c:
                    cells.clear()
                    cells.extend([[True for i in range(resolution_width)] for j in range(resolution_height)])
                    update_path = True

                if event.key == pygame.K_s:
                    # set the starting location to the mouse location
                    x, y = pygame.mouse.get_pos()
                    
                    # calculate the graph position from the mouse position
                    cell_width = graph_width / resolution_width
                    cell_height = graph_height / resolution_height
                    
                    base_x = (screen_width - graph_width) / 2
                    base_y = screen_height - (screen_height - graph_height ) / 2
                    
                    cell_x = (x - base_x) / cell_width
                    cell_y = (base_y - y) / cell_height

                    
                    start_location = (int(cell_x), int(cell_y))

                    update_path = True
                    
                    print("Set start location to: ", start_location)

                if event.key == pygame.K_e:
                    # set the ending location to the mouse location
                    x, y = pygame.mouse.get_pos()
                    
                    # calculate the graph position from the mouse position
                    cell_width = graph_width / resolution_width
                    cell_height = graph_height / resolution_height
                    
                    base_x = (screen_width - graph_width) / 2
                    base_y = screen_height - (screen_height - graph_height ) / 2
                    
                    cell_x = (x - base_x) / cell_width
                    cell_y = (base_y - y) / cell_height

                    
                    end_location = (int(cell_y), int(cell_x))

                    update_path = True
                    
                    print("Set end location to: ", end_location)

            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = event.pos

                cell_width = graph_width / resolution_width
                cell_height = graph_height / resolution_width

                base_x = (screen_width - graph_width) / 2
                base_y = screen_height - (screen_height - graph_height ) / 2

                for i in range(resolution_height):
                    for j in range(resolution_width):
                        cell_x = base_x + (j * cell_width)
                        cell_y = base_y - ((i + 1)* cell_height)

                        if x >= cell_x and x <= cell_x + cell_width and y >= cell_y and y <= cell_y + cell_height:
                            mouse_fill_mode = not cells[i][j]
                            mouse_down = True
                            cells[i][j] = mouse_fill_mode
                            update_path = True
                            break

            if event.type == pygame.MOUSEMOTION:
                if mouse_down:
                    x, y = event.pos

                    cell_width = graph_width / resolution_width
                    cell_height = graph_height / resolution_width

                    base_x = (screen_width - graph_width) / 2
                    base_y = screen_height - (screen_height - graph_height ) / 2

                    for i in range(resolution_height):
                        for j in range(resolution_width):
                            cell_x = base_x + (j * cell_width)
                            cell_y = base_y - ((i + 1) * cell_height)

                            if x >= cell_x and x <= cell_x + cell_width and y >= cell_y and y <= cell_y + cell_height:
                                cells[i][j] = mouse_fill_mode
                                update_path = True

            if event.type == pygame.MOUSEBUTTONUP:
                mouse_down = False

                    
        screen.fill((0, 0, 0))

        cell_width = graph_width / resolution_width
        cell_height = graph_height / resolution_width

        base_x = (screen_width - graph_width) / 2
        base_y = screen_height - (screen_height - graph_height ) / 2

        
        for i in range(resolution_height):
            for j in range(resolution_width):
                x = base_x + (j * cell_width)
                y = base_y - (i * cell_height)
                color = (0, 255, 0) if cells[i][j] else (255, 0, 0)

                pygame.draw.rect( screen, color, (x, y - cell_height, cell_width, cell_height)) 


        if update_path:
            pf = pathfinder.Pathfinder(resolution_width, resolution_height, cells)
            path = pf.generate_path(start_location, end_location)
            update_path = False

        # draw path
        x_scale = graph_width / resolution_width 
        y_scale = graph_height / resolution_height 

        color = (0, 0, 255)

        for pose in path:
            pygame.draw.circle(screen, color, (base_x + pose[0] * x_scale + (0.5 * x_scale), base_y - pose[1] * y_scale - (0.5 * y_scale)), 5)






        label_x_step = graph_width / horiz_label_number
        label_y_step = graph_height / vert_label_number

        # Set font
        
        font = pygame.font.Font(None, 16)

        for i in range(horiz_label_number + 1):
            x = base_x + i * label_x_step
            pygame.draw.line(screen, (255, 255, 255), (x, base_y), (x, base_y - graph_height))

            text = font.render(str(round((i / horiz_label_number) * angle_size_degs)) + "°", True, (255, 255, 255))

            text_rect = text.get_rect(center=(x, base_y + 20))
            screen.blit(text, text_rect)




        for i in range(vert_label_number + 1):
            y = base_y - i * label_y_step
            pygame.draw.line(screen, (255, 255, 255), (base_x, y), (base_x + graph_width, y))

            text = font.render(str(round((i / vert_label_number) * height_inches, 2)) + "in", True, (255, 255, 255))

            text_rect = text.get_rect(center=(base_x - 20, y))
            screen.blit(text, text_rect)
        
        
        pygame.display.flip()

    save_cells(cells, "cells.txt")
    
        
    pygame.quit()




if __name__ == "__main__":
    main()
