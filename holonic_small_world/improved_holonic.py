import random
import networkx as nx
import matplotlib.pyplot as plt

class Agent:
    def __init__(self, id):
        self.id = id
        self.status = 'active'
        self.task = None
        self.connections = []  # Direct connections to other agents

    def assign_task(self, task, visited=None):
        if visited is None:
            visited = set()
            
        # Don't process this agent again if already visited
        if self.id in visited:
            return False
            
        visited.add(self.id)
        
        if self.status == 'active' and self.task is None:
            self.task = task
            return True
            
        # Try to delegate to connected agents
        for connection in self.connections:
            if connection.is_active() and connection.task is None:
                if connection.assign_task(task, visited):
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
    
    def add_connection(self, agent):
        if agent not in self.connections:
            self.connections.append(agent)

class Holon:
    def __init__(self, id, members=None):
        self.id = id
        self.members = members if members else []
        self.connections = []  # Connections to other holons

    def assign_task(self, task, visited=None):
        if visited is None:
            visited = set()
            
        # Don't process this holon again if already visited
        if self.id in visited:
            return False
            
        visited.add(self.id)
        
        # Try internal members first
        for m in self.members:
            if isinstance(m, Agent) and m.is_active():
                if m.assign_task(task, visited):
                    return True
            elif isinstance(m, Holon):
                if m.assign_task(task, visited):
                    return True
                
        # Try connections if internal assignment failed
        for holon in self.connections:
            if holon.assign_task(task, visited):
                return True
                
        return False

    def fail(self):
        for m in self.members:
            if isinstance(m, Agent):
                m.fail()
            elif isinstance(m, Holon):
                m.fail()
    
    def add_connection(self, holon):
        if holon not in self.connections:
            self.connections.append(holon)

def build_holonic_structure(num_agents, agents_per_holon):
    agents = [Agent(f"A{i}") for i in range(num_agents)]
    holons = []
    for i in range(0, num_agents, agents_per_holon):
        holon = Holon(f"H{i//agents_per_holon}", agents[i:i+agents_per_holon])
        holons.append(holon)
    root_holon = Holon("ROOT", holons)
    return root_holon, agents, holons

def add_small_world_connections(agents, holons, rewiring_prob=0.2):
    # Add local connections first (connect agents within same holon)
    for holon in holons:
        agent_members = [m for m in holon.members if isinstance(m, Agent)]
        for i, agent in enumerate(agent_members):
            # Connect to next agent in same holon (ring structure)
            next_agent = agent_members[(i + 1) % len(agent_members)]
            agent.add_connection(next_agent)
    
    # Create more connections within holon (triangle formation)
    for holon in holons:
        agent_members = [m for m in holon.members if isinstance(m, Agent)]
        for i, agent in enumerate(agent_members):
            for j in range(1, 3):  # Connect to next 2 neighbors
                next_agent = agent_members[(i + j) % len(agent_members)]
                agent.add_connection(next_agent)
    
    # Add random long-range connections (small-world property)
    for agent in agents:
        if random.random() < rewiring_prob:
            # Create connection to random agent
            random_agent = random.choice(agents)
            # Avoid self-connections
            while random_agent == agent or random_agent in agent.connections:
                random_agent = random.choice(agents)
            agent.add_connection(random_agent)
    
    # Connect holons in a ring with some random connections
    for i, holon in enumerate(holons):
        # Connect to next holon
        next_holon = holons[(i + 1) % len(holons)]
        holon.add_connection(next_holon)
        
        # Add random holon connections
        if random.random() < rewiring_prob:
            random_holon = random.choice(holons)
            while random_holon == holon or random_holon in holon.connections:
                random_holon = random.choice(holons)
            holon.add_connection(random_holon)

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

def calculate_network_metrics(agents, holons):
    # Create a graph to calculate metrics
    G = nx.Graph()
    for agent in agents:
        G.add_node(agent.id)
    
    # Add agent connections
    for agent in agents:
        for connection in agent.connections:
            G.add_edge(agent.id, connection.id)
    
    # Calculate metrics
    avg_path_length = nx.average_shortest_path_length(G) if nx.is_connected(G) else -1
    clustering_coef = nx.average_clustering(G)
    
    return {
        "avg_path_length": avg_path_length,
        "clustering_coefficient": clustering_coef
    }

def compare_architectures(num_agents=100, agents_per_holon=10, 
                          num_rounds=20, tasks_per_round=20, rewiring_probs=None):
    if rewiring_probs is None:
        rewiring_probs = [0, 0.1, 0.2, 0.3, 0.4]
        
    results = []
    
    for fail_rate in [0.90, 0.93, 0.95, 0.97, 0.99]:
        fail_results = []
        
        for rewiring_prob in rewiring_probs:
            # Build structure
            root_holon, agents, holons = build_holonic_structure(num_agents, agents_per_holon)
            
            # Add small-world connections if enabled
            if rewiring_prob > 0:
                add_small_world_connections(agents, holons, rewiring_prob)
            
            # Run simulation
            completed, total = simulate_batch(root_holon, agents, 
                                             num_rounds=num_rounds,
                                             tasks_per_round=tasks_per_round,
                                             fail_rate=fail_rate)
            
            # Calculate metrics
            if rewiring_prob > 0:
                metrics = calculate_network_metrics(agents, holons)
                network_type = f"Small-world (p={rewiring_prob})"
            else:
                metrics = {"avg_path_length": -1, "clustering_coefficient": 0}
                network_type = "Hierarchical only"
            
            fail_results.append({
                "network_type": network_type,
                "completed": completed,
                "total": total,
                "completion_rate": completed/total,
                "metrics": metrics
            })
            
        results.append({
            "fail_rate": fail_rate,
            "results": fail_results
        })
    
    return results

def visualize_network(agents):
    G = nx.Graph()
    for agent in agents:
        G.add_node(agent.id)
    for agent in agents:
        for conn in agent.connections:
            G.add_edge(agent.id, conn.id)
    plt.figure(figsize=(10, 10))
    nx.draw(G, node_size=50)
    plt.savefig("network.png")
    plt.close()

if __name__ == "__main__":
    print("Comparing hierarchical vs small-world holonic architectures")
    results = compare_architectures(
        num_agents=100, 
        agents_per_holon=10,
        num_rounds=20,
        tasks_per_round=20
    )
    
    # Display results
    for fail_scenario in results:
        fail_rate = fail_scenario["fail_rate"]
        print(f"\n--- Failure rate: {fail_rate*100}% ---")
        for res in fail_scenario["results"]:
            print(f"{res['network_type']}: {res['completed']}/{res['total']} tasks " +
                 f"({res['completion_rate']*100:.1f}%)")
            if res['metrics']['avg_path_length'] > 0:
                print(f"  Avg path length: {res['metrics']['avg_path_length']:.2f}")
                print(f"  Clustering coefficient: {res['metrics']['clustering_coefficient']:.2f}")
    
    # Visualize different network structures
    print("\nGenerating network visualizations...")
    for rewiring_prob in [0, 0.1, 0.2, 0.3, 0.4]:
        root_holon, agents, holons = build_holonic_structure(100, 10)
        if rewiring_prob > 0:
            add_small_world_connections(agents, holons, rewiring_prob)
        visualize_network(agents)
        # Rename to include the rewiring probability
        import os
        os.rename("network.png", f"network_p{rewiring_prob}.png")
        print(f"Created visualization for p={rewiring_prob}")