import random
from small_world import create_small_world_topology
from holonic_hierarchy import build_holonic_structure

def simulate(root_holon, agents, num_tasks=20, fail_rate=0.1):
    completed = 0

    for i in range(num_tasks):
        task = f"T{i}"

        # Step 1: Reset tasks
        for a in agents:
            a.reset_task()

        # Step 2: Random failures
        for a in agents:
            if random.random() < fail_rate:
                a.fail()

        # Step 3: Try assigning task
        success = root_holon.assign_task(task)
        if success:
            completed += 1

    return completed, num_tasks


def simulate_batch(root_holon, agents, num_rounds=10, tasks_per_round=5, fail_rate=0.2):
    completed = 0
    for i in range(num_rounds):
        # Reset
        for a in agents:
            a.reset_task()
            if random.random() < fail_rate:
                a.fail()
        
        for t in range(tasks_per_round):
            task = f"T{i}_{t}"
            if root_holon.assign_task(task):
                completed += 1
    return completed, num_rounds * tasks_per_round


def sweep_failure_rates(root_holon_factory, agents_factory, rates, num_tasks=50):
    results = []
    for rate in rates:
        root_holon, agents = root_holon_factory(), agents_factory()
        completed, _ = simulate(root_holon, agents, num_tasks=num_tasks, fail_rate=rate)
        results.append((rate, completed))
        print(f"Failure rate: {rate:.2f} → Completed: {completed}/{num_tasks}")
    return results


def make_factories(num_agents=100, agents_per_holon=10):
    def root_factory():
        return build_holonic_structure(num_agents, agents_per_holon)[0]
    def agents_factory():
        return build_holonic_structure(num_agents, agents_per_holon)[1]
    return root_factory, agents_factory


# if __name__ == "__main__":
#     num_agents = 100
#     agents_per_holon = 10
    
#     topology = create_small_world_topology(num_agents)
#     root_holon, agents = build_holonic_structure(num_agents, agents_per_holon)
    
#     completed, total = simulate(root_holon, agents, num_tasks=50, fail_rate=0.05)
    
#     print(f"Completed tasks: {completed}/{total}")

if __name__ == "__main__":
    root_factory, agents_factory = make_factories()
    fail_rates = [0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5]
    sweep_failure_rates(root_factory, agents_factory, fail_rates)