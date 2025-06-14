class SoS:
    def __init__(self):
        self.S = set() # set of constituent systems
        self.G = set() # set of goals

    def add_system(self, system):
        self.S.add(system)

    def remove_system(self, system):
        if system in self.S:
            self.S.remove(system)
        else:
            raise ValueError("CS not found in the set of constituent systems.")

    def add_goal(self, goal):
        self.G.add(goal)

    def remove_goal(self, goal):
        if goal in self.G:
            self.G.remove(goal)
        else:
            raise ValueError("Goal not found in the set of goals.")

    def __repr__(self):
        return f"SoS(S={self.S}, goals={self.G})"

