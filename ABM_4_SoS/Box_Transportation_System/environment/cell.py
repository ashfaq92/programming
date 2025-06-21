class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.robot = None
        self.nest = None
        self.box = None

    def is_empty(self):
        return self.robot is None and self.box is None and self.nest is None
    
    def add_robot(self, robot):
        if self.robot is None:     # Robot can coexist with nest
            self.robot = robot
        else:
            raise ValueError("Cannot add robot to non-empty cell.")

    def add_box(self, box):
        if self.robot is None and self.box is None:     # box can coexist with nest
            self.box = box
            self.box.set_position(self.x, self.y)
        else:
            raise ValueError("Cannot add box to non-empty cell.")

    def remove_box(self):
        if self.box is not None:
            # self.box.clear_position()
            self.box = None
        else:
            raise ValueError("Cannot remove box from cell.")

    def add_nest(self, nest):
        if self.is_empty(): # nest require completely empty cells
            self.nest = nest
        else:
            raise ValueError("Cannot add nest to non-empty cell.")
    
    def remove_robot(self):
        self.robot = None



    def remove_nest(self):
        self.nest = None