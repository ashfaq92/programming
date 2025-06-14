import random
from agent import Agent

class Holon:
    def __init__(self, id, members=None):
        self.id = id
        self.members = members if members else []

    def assign_task(self, task):
        for m in self.members:
            if isinstance(m, Agent) and m.is_active():
                if m.assign_task(task):
                    return True
            elif isinstance(m, Holon):
                if m.assign_task(task):
                    return True
        return False

    def fail_random_member(self):
        if not self.members:
            return
        victim = random.choice(self.members)
        if isinstance(victim, Agent):
            victim.fail()
        elif isinstance(victim, Holon):
            victim.fail_random_member()
    
    def fail(self):
        for m in self.members:
            if isinstance(m, Agent):
                m.fail()
            elif isinstance(m, Holon):
                m.fail()
