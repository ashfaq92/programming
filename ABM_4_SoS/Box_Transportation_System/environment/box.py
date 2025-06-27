import utils


class Box:
    def __init__(self, c):
        self.color = utils.validate_color(c)
        self.status = None
        self.position = None
        self.holder = None

    def set_position(self, pos):
        """Set and cache position"""
        # todo validate position
        x, y = pos
        if utils.validate_position((x, y)):
            self.position = (x, y)
        else:
            raise ValueError('invalid position given to box')


    def clear_position(self):
        """Clear cached position when box is picked up"""
        self.position = None
    
    def set_status(self, s):
        if s in utils.BOX_STATUSES:
            self.status = s
        else: 
            raise ValueError(f"Invalid box status: {s}")