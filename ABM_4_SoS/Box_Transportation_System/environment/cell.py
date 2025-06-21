class Cell:
    # todo: check if nest and box overlap
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.robot = None
        self.nest = None
        self.box = None

    def is_empty(self):
        return self.robot is None and self.box is None and self.nest is None
    
    def add_robot(self, robot):
        if self.robot is None:
            self.robot = robot
        else:
            raise ValueError("Cell already has a robot")

    def add_box(self, box):
        if self.box is None:
            self.box = box
        else:
            raise ValueError("Cell already has a box")

    def add_nest(self, nest):
        if self.nest is None:
            self.nest = nest
        else:
            raise ValueError("Cell already has a nest")
    
    def remove_robot(self):
        self.robot = None

    def remove_box(self):
        self.box = None

    def remove_nest(self):
        self.nest = None