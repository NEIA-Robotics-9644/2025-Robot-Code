def dist(x, y, x2, y2):
    return ((x - x2) ** 2 + (y - y2) ** 2) ** 0.5

class Cell:
    parent = None
    x = 0
    y = 0
    distance = 0

    def __init__(self, x, y, parent, target_x, target_y):
        self.x = x
        self.y = y
        self.parent = parent
        self.distance = dist(x, y, target_x, target_y)

class Pathfinder:
    
    cells = [[]]

    resolution_width = 0
    resolution_height = 0
    
    def __init__(self, resolution_width, resolution_height, cells):
        self.cells = cells
        self.resolution_width = resolution_width
        self.resolution_height = resolution_height

    def is_cell_open(self, x, y, found_cells):
        for cell in found_cells:
            if abs(cell.x - x) <= 0.1 and abs(cell.y - y) <= 0.1:
                return False

        if y >= self.resolution_height or y < 0 or x >= self.resolution_width or x < 0:
            return False

        return self.cells[y][x]


        
    def generate_path(self, current_pose, target_pose):
        
        current_height = current_pose[0]
        current_angle = current_pose[1]
        target_height = target_pose[0]
        target_angle = target_pose[1]

        # A* pathfinding

        
        cell_queue = []

        found_cells = []

        
        cell_queue.append(Cell(current_height, current_angle, None, target_height, target_angle))

        path_found = False
        
        end_cell = None
        
        while not path_found:
            if len(cell_queue) == 0:
                print("No path found")
                return []

                
            cell_queue.sort(key=lambda x: x.x)

            cell_to_expand = cell_queue[0]

            x = cell_to_expand.x
            y = cell_to_expand.y

            if abs(x - target_angle) <= 0.1 and abs(y - target_height) <= 0.1:
                path_found = True
                end_cell = cell_to_expand
                
            cell_queue = cell_queue[1:]

            if self.is_cell_open(x, y + 1, found_cells):
                new_cell = Cell(x, y + 1, cell_to_expand, target_height, target_angle)
                found_cells.append(new_cell)
                cell_queue.append(new_cell)

            if self.is_cell_open(x, y - 1, found_cells):
                new_cell = Cell(x, y - 1, cell_to_expand, target_height, target_angle)
                found_cells.append(new_cell)
                cell_queue.append(new_cell)
            
            if self.is_cell_open(x + 1, y, found_cells):
                new_cell = Cell(x + 1, y, cell_to_expand, target_height, target_angle)
                found_cells.append(new_cell)
                cell_queue.append(new_cell)

            if self.is_cell_open(x - 1, y, found_cells):
                new_cell = Cell(x - 1, y, cell_to_expand, target_height, target_angle)
                found_cells.append(new_cell)
                cell_queue.append(new_cell)
        
        # Follow back to get the path
        
        path = []
        
        back_to_start = False

        current_cell = end_cell
        
        while not back_to_start:
            path.append((current_cell.x, current_cell.y))
            if current_cell.parent is None:
                back_to_start = True
            current_cell = current_cell.parent

        path.reverse()

        print("generated path")
            
        return path
