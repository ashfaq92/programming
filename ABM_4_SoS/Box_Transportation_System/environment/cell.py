import utils


class Cell:
    def __init__(self, position):
        self.position = position
        self.robot = None
        self.nest = None
        self.box = None

    def is_empty(self):
        return self.robot is None and self.box is None and self.nest is None
    
    def add_nest(self, nest):
        if self.is_empty(): # nest require completely empty cells
            self.nest = nest
        else:
            raise ValueError("Cannot add nest to non-empty cell.")

    def add_box(self, box):
        # if self.robot is None and self.box is None:     # box can coexist with nest
        if self.box is None:   
            self.box = box
            self.box.set_position(self.position)
        else:
            raise ValueError("Cannot add box to non-empty cell.")

    def add_robot(self, robot):
        if utils.ROBOT_PASSTHROUGH:     # allowing multiple robots to exist on one cell
            self.robot = robot
        else:
            if self.robot is None:     # However, the robot can coexist with nest
                self.robot = robot
            else:
                raise ValueError("Cannot add robot to non-empty cell.")

    def remove_nest(self):
        self.nest = None
    
    def remove_robot(self):
        self.robot = None

    def remove_box(self):
        self.box = None