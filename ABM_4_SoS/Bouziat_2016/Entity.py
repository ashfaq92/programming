class Entity:
    def __init__(self, entity_id):
        self.entity_id = entity_id
        self.F = set()  # Set of functionalities
        self.R = set()  # Set of resources
        self.G = set()  # Set of goals
        self.L = set()  # Set of links with entities or component-systems

    def add_functionality(self, functionality):
        self.F.add(functionality)

    def add_resource(self, resource):
        self.R.add(resource)

    def add_goal(self, goal):
        self.G.add(goal)

    def add_link(self, link):
        self.L.add(link)

    def __str__(self):
        return f"E{self.entity_id} = {{F={len(self.F)}, R={len(self.R)}, G={len(self.G)}, L={len(self.L)}}}"