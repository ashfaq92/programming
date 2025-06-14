import networkx as nx

def create_small_world_topology(num_agents, k=4, p=0.2):
    G = nx.watts_strogatz_graph(num_agents, k, p)
    return G
