import utils


class Box:
    def __init__(self, c):
        self.color = utils.validate_color(c)
        self.status = None
        self.position = None

    def set_position(self, x, y):
        """Set and cache position"""
        self.position = (x, y)

    def clear_position(self):
        """Clear cached position when box is picked up"""
        self.position = None
    
    def set_status(self, s):
        if s in utils.BOX_STATUSES:
            self.status = s
        else: 
            raise ValueError(f"Invalid box status: {s}")