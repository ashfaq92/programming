import utils

class Nest:
    # TODO: check if nest position has problem
    def __init__(self, color, position):
        self.color = utils.validate_color(color)
        self.deposited_boxes = []  # List of deposited boxes
        self.position = position

