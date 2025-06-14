class Agent:
    def __init__(self, id):
        self.id = id
        self.status = 'active'
        self.task = None

    def assign_task(self, task):
        if self.status == 'active' and self.task is None:
            self.task = task
            return True
        return False

    def fail(self):
        self.status = 'failed'
        self.task = None

    def recover(self):
        self.status = 'active'

    def reset_task(self):
        self.task = None

    def is_active(self):
        return self.status == 'active'
