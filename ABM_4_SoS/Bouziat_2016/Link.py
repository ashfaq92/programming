class Link:
    def __init__(self, from_system: str, to_system: str):
        self.from_system = from_system
        self.to_system = to_system

    def __repr__(self):
        return f"Link({self.from_system} -> {self.to_system})"