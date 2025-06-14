from agent import Agent
from holon import Holon

def build_holonic_structure(num_agents, agents_per_holon):
    agents = [Agent(f"A{i}") for i in range(num_agents)]
    holons = []
    for i in range(0, num_agents, agents_per_holon):
        holon = Holon(f"H{i//agents_per_holon}", agents[i:i+agents_per_holon])
        holons.append(holon)
    root_holon = Holon("ROOT", holons)
    return root_holon, agents
