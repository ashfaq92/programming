import random

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

    def fail(self):
        for m in self.members:
            if isinstance(m, Agent):
                m.fail()
            elif isinstance(m, Holon):
                m.fail()

def build_holonic_structure(num_agents, agents_per_holon):
    agents = [Agent(f"A{i}") for i in range(num_agents)]
    holons = []
    for i in range(0, num_agents, agents_per_holon):
        holon = Holon(f"H{i//agents_per_holon}", agents[i:i+agents_per_holon])
        holons.append(holon)
    root_holon = Holon("ROOT", holons)
    return root_holon, agents

def simulate_batch(root_holon, agents, num_rounds=10, tasks_per_round=5, fail_rate=0.2):
    completed = 0
    total_tasks = num_rounds * tasks_per_round

    for i in range(num_rounds):
        # Reset tasks and simulate failures
        for a in agents:
            a.reset_task()
            if random.random() < fail_rate:
                a.fail()
            else:
                a.recover()  # Agents recover if not failing this round

        # Assign multiple tasks per round
        for t in range(tasks_per_round):
            task = f"T{i}_{t}"
            if root_holon.assign_task(task):
                completed += 1

    return completed, total_tasks

# Add connections between agents or holons beyond the hierarchy
def add_small_world_connections(agents, rewiring_prob=0.1):
    for agent in agents:
        # Add local connections
        # Add some random long-distance connections
        if random.random() < rewiring_prob:
            random_agent = random.choice(agents)
            # Create connection between agent and random_agent

if __name__ == "__main__":
    num_agents = 100
    agents_per_holon = 10
    root_holon, agents = build_holonic_structure(num_agents, agents_per_holon)

    completed, total = simulate_batch(root_holon, agents, num_rounds=20, tasks_per_round=5, fail_rate=0.3)
    print(f"Completed tasks: {completed}/{total}")
