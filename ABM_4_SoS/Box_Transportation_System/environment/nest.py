import utils

class Nest:
    # TODO: check if nest position has problem
    def __init__(self, c, x, y):
        self.color = utils.validate_color(c)
        self.deposited_boxes = []  # List of deposited boxes
        self.position = (x, y)

