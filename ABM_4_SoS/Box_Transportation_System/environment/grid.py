import utils
from .box import Box
from .cell import Cell
from .nest import Nest


class Grid:
    def __init__(self, width=50, height=50):
        self.width = width
        self.height = height
        self.cells = []
        self.nests = []
        self.boxes = []
        self.robots = []
        # Initializations
        self._initialize_cells()
        # self._initialize_nests_randomly()
        self._initialize_nests_fixed_equidistant()

    def _initialize_cells(self):
        self.cells = [[Cell(x,y) for x in range(self.width)] for y in range(self.height)]  #2D grid

    def _initialize_nests_randomly(self):
        """initialize three nests (red, green, blue) equidistant from each other"""
        colors = utils.COLORS.copy()
        utils.seeded_rand.shuffle(colors)

        placed_nests = 0
        attempts = 0
        attempts_max = 1000
        
        while placed_nests < len(colors) and attempts < attempts_max:
            x = utils.seeded_rand.randint(0, self.width - 1)
            y = utils.seeded_rand.randint(0, self.height - 1)

            # Check if cell is empty 
            if self.cells[y][x].is_empty():
                nest = Nest(colors[placed_nests], x, y)
                self.cells[y][x].add_nest(nest)
                self.nests.append(nest)
                placed_nests += 1
            attempts += 1
        
        if placed_nests < len(colors):
            raise ValueError(f"Could only place {placed_nests} out of {len(colors)} nests")

    def _initialize_nests_fixed_equidistant(self):
        """Initialize nests at predetermined equidistant positions"""
        colors = utils.COLORS.copy()
        utils.seeded_rand.shuffle(colors)  # Still randomize color assignment

        # Predetermined positions forming equilateral triangle (for 50x50 grid)
        fixed_positions = [
            (25, 15),  # Top
            (15, 35),  # Bottom-left
            (35, 35)  # Bottom-right
        ]

        for i, (x, y) in enumerate(fixed_positions):
            if self.cells[y][x].is_empty():
                nest = Nest(colors[i], x, y)
                self.cells[y][x].add_nest(nest)
                self.nests.append(nest)
            else:
                raise ValueError(f"Predetermined nest position ({x}, {y}) is not empty!")

    def initialize_boxes(self, n):
        """Initialize n boxes randomly on the grid"""
        # self.boxes = []
        boxes_placed = 0
        for _ in range(n):
            random_color = utils.seeded_rand.choice(utils.COLORS)
            box = Box(c=random_color)
            # find random empty cell
            attempts = 0
            attempts_max = 5000
            while attempts < attempts_max:   # prevent infinite loop
                cell = utils.seeded_rand.choice([cell for row in self.cells for cell in row])
                if cell.is_empty():
                    box.set_status("INITIALIZED")
                    cell.add_box(box)
                    self.boxes.append(box)
                    boxes_placed += 1
                    break   # Break after placing one box
                attempts += 1

            if attempts == attempts_max:
                print(f"Warning: Could not place box {boxes_placed + 1} after {attempts_max} attempts")
                break
    
    def add_robot(self, robot, x=None, y=None):
        """Add a robot to the grid at specified or random position"""

        if x is not None and y is not None:
            if self.cells[y][x].is_empty():
                self.cells[y][x].add_robot(robot)
                robot.position = (x, y)
                self.robots.append(robot)
                return True
        else:
            # Find random empty position
            attempts = 0
            while attempts < 3000:
                x = utils.seeded_rand.randint(0, self.width - 1)
                y = utils.seeded_rand.randint(0, self.height - 1)
                if self.cells[y][x].is_empty():
                    self.cells[y][x].add_robot(robot)
                    robot.position = (x, y)
                    self.robots.append(robot)
                    return True
                attempts += 1
        return False