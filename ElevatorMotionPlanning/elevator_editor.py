import pygame

class Constraint:
    angle_degs = 0

    def __init__(self, angle_degs):
        self.angle_degs = angle_degs

class Selectable:
    x = 0
    y = 0
    radius = 0
    color = (255, 0, 0)

    constraint = None
    
    def __init__(self, x, y, radius, constraint, color):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.constraint = constraint
        
    def is_inside(self, x, y):
        if (abs(x - self.x) <= self.radius / 2) and (abs(y - self.y) <= (self.radius / 2)):
            return True

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def update_constrant(self, scale):
        self.constraint.angle_degs = self.x * scale

    def draw(self, screen, tf):
        # Draw the selectable on the screen
        tf.draw_rect(screen, self.color, self.x - (self.radius / 2), self.y - (self.radius / 2), self.radius, self.radius)
        
        # write the angle to the screen
        font = pygame.font.Font(None, 14)
        text = font.render(str(round(self.constraint.angle_degs, 2)), True, (255, 255, 255))
        screen_x, screen_y = tf.elevator_to_screen(self.x, self.y)

        text_rect = text.get_rect(center=(screen_x, screen_y))

        if self.constraint.angle_degs > 0:
            text_rect = text.get_rect(center=(screen_x + text_rect.width, screen_y))
        else:
            text_rect = text.get_rect(center=(screen_x - text_rect.width, screen_y))

        screen.blit(text, text_rect)

SCALE = 180

def save_constraints(min_constraints, max_constraints, filename):
    with open(filename, 'w') as f:
        for i in range(len(min_constraints)):
            f.write(f"{min_constraints[i].angle_degs},{max_constraints[i].angle_degs}\n")

def load_constraints(filename):

    min_constraints.clear()
    max_constraints.clear()
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            min_angle, max_angle = line.split(',')
            min_constraints.append(Constraint(float(min_angle)))
            max_constraints.append(Constraint(float(max_angle)))
            print("Loaded: ", min_angle, max_angle)
            
    selectables.clear()
    for i in range(len(min_constraints)):
        selectables.append(Selectable(max_constraints[i].angle_degs / SCALE, float(i) / (len(min_constraints) - 1), 0.02, max_constraints[i], (0, 255, 0)))
        selectables.append(Selectable(min_constraints[i].angle_degs / SCALE, float(i) / (len(min_constraints) - 1), 0.02, min_constraints[i], (0, 0, 255)))

max_constraints = []

min_constraints = []

constraint_number = 50 

for i in range(constraint_number):
    max_constraints.append(Constraint(0))
    min_constraints.append(Constraint(0.4))

selectables = []

for i in range(constraint_number):
    selectables.append(Selectable(max_constraints[i].angle_degs, float(i) / (constraint_number - 1),  0.02, max_constraints[i], (0, 255, 0)))
    selectables.append(Selectable(min_constraints[i].angle_degs, float(i) / (constraint_number - 1), 0.02, min_constraints[i], (0, 0, 255)))

class Transform: 
    world_width = 800
    world_height = 600

    elevator_top_offset = 50
    elevator_bottom_offset = 50

    elevator_screen_height = world_height - elevator_top_offset - elevator_bottom_offset
    
    def __init__(self, world_width, world_height, elevator_top_offset, elevator_bottom_offset):
        self.world_width = world_width
        self.world_height = world_height
        self.elevator_top_offset = elevator_top_offset
        self.elevator_bottom_offset = elevator_bottom_offset
        self.elevator_screen_height = world_height - elevator_top_offset - elevator_bottom_offset
        
    def elevator_to_screen(self, x, y):
        # Transform elevator coordinates to screen coordinates

        screen_x = ((x) * (self.world_height - self.elevator_bottom_offset - self.elevator_top_offset) ) + self.world_width / 2
        screen_y = self.elevator_top_offset + ((1 - y) * (self.world_height - self.elevator_top_offset - self.elevator_bottom_offset))

        return (screen_x, screen_y)

    def screen_to_elevator(self, x, y):
        # Transform screen coordinates to elevator coordinates

        elevator_x = ((x - (self.world_width / 2)) / (self.world_height - self.elevator_bottom_offset - self.elevator_top_offset))
        elevator_y = 1 - ((y - self.elevator_top_offset) / (self.world_height - self.elevator_top_offset - self.elevator_bottom_offset))

        return (elevator_x, elevator_y)
        
    def draw_rect(self, screen, color, x, y, width, height):
        # Draw a rectangle on the screen
        screen_x, screen_y = self.elevator_to_screen(x, y)
        screen_x2, screen_y2 = self.elevator_to_screen(x + width, y + height)

        rect = pygame.Rect(screen_x, screen_y2, screen_x2 - screen_x, screen_y - screen_y2)

        pygame.draw.rect(screen, color, rect)
    
# UI for editing constraints with pygame

# Start the loop

def main():

    screen_size = (800, 600)

    screen_width = screen_size[0]
    screen_height = screen_size[1]

    tf = Transform(screen_width, screen_height, 50, 50)

    load_constraints("constraints.txt")

    pygame.init()
    screen = pygame.display.set_mode(screen_size)
    pygame.display.set_caption("Elevator Motion Planning")
    
    running = True

    selected = None
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                x, y = event.pos

                e_x, e_y = tf.screen_to_elevator(x, y)

                print("Mouse clicked at: ", e_x, e_y)
                for selectable in selectables:
                    print("Checking if inside: ", selectable.x, selectable.y, selectable.radius)
                    if selectable.is_inside(e_x, e_y):
                        print("Selected")
                        selected = selectable
                        break
                    
            if event.type == pygame.MOUSEBUTTONUP:
                selected = None

            if event.type == pygame.MOUSEMOTION:
                if selected is not None:
                    x, y = event.pos
                    e_x, e_y = tf.screen_to_elevator(x, y)
                    selected.set_position(e_x, selected.y)
                    save_constraints(min_constraints, max_constraints, "constraints.txt")
                    
            if event.type == pygame.KEYDOWN:
               if event.key == pygame.K_ESCAPE:
                    running = False
        
        screen.fill((0, 0, 0))

        # Draw the elevator
        tf.draw_rect(screen, (255, 255, 255), -0.0005, 0, 0.005, 1)
        
        # Draw the selectables
        for selectable in selectables:
            selectable.update_constrant(SCALE)
            selectable.draw(screen, tf)
       
        pygame.display.flip()

    pygame.quit()

    save_constraints(min_constraints, max_constraints, "constraints.txt")

if __name__ == "__main__":
    main()