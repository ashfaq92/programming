from Box_Transportation_System import utils


class Box:
    def __init__(self, c):
        self.color = utils.validate_color(c)
        self.status = None
    
    def set_status(self, s):
        if s in utils.BOX_STATUSES:
            self.status = s
        else: 
            raise ValueError(f"Invalid box status: {s}")