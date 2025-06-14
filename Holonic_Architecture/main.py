import numpy as np
import matplotlib.pyplot as plt
import random
import time
from collections import deque

random.seed(42)
np.random.seed(42)

# ===================
# 1. Simulation Setup
# ===================
class Agent:
    def __init__(self, agent_id, skills):
        self.id = agent_id
        self.skills = skills
        self.available = True
        self.failed = False
        self.current_task = None
        self.work_remaining = 0
    
    def bid(self, task):
        if self.failed or not self.available:
            return None
        for skill, req in task.requirements.items():
            if self.skills.get(skill, 0) < req:
                return None
        return sum(self.skills[skill] for skill in task.requirements)
    
    def assign_task(self, task):
        self.current_task = task
        self.available = False
        self.work_remaining = task.complexity
    
    def work(self):
        if self.failed or self.work_remaining <= 0 or not self.current_task:
            return False
            
        self.work_remaining -= 1
        if self.work_remaining == 0:
            self.available = True
            return True  # Task completed
        return False
    
    def fail(self):
        self.failed = True
        self.available = False
        self.work_remaining = 0

class Holon:
    def __init__(self, holon_id, members):
        self.id = holon_id
        self.members = members
        self.skills = self._aggregate_skills()
        self.failed = False
    
    def _aggregate_skills(self):
        skills = {}
        for member in self.members:
            if member.failed:
                continue
            if isinstance(member, Agent):
                for skill, val in member.skills.items():
                    if skill not in skills or val > skills[skill]:
                        skills[skill] = val
            else:
                for skill, val in member.skills.items():
                    if skill not in skills or val > skills[skill]:
                        skills[skill] = val
        return skills
    
    def assign_task(self, task):
        if self.failed:
            return None, None
            
        candidates = []
        for member in self.members:
            if member.failed:
                continue
                
            if isinstance(member, Agent):
                if member.available:
                    bid_val = member.bid(task)
                    if bid_val is not None:
                        candidates.append((member, bid_val))
            else:
                # Only delegate to sub-holons that can handle the task
                if all(member.skills.get(skill, 0) >= req for skill, req in task.requirements.items()):
                    candidate, bid_val = member.assign_task(task)
                    if candidate:
                        candidates.append((candidate, bid_val))
        
        if not candidates:
            return None, None
        
        best_candidate = max(candidates, key=lambda x: x[1])
        return best_candidate
    
    def update_availability(self):
        self.skills = self._aggregate_skills()
        
    def fail(self):
        self.failed = True
        for member in self.members:
            member.fail()

class Task:
    def __init__(self, task_id, requirements, complexity):
        self.id = task_id
        self.requirements = requirements
        self.complexity = complexity
        self.creation_time = 0
        self.completion_time = None
        self.attempts = 0  # Track assignment attempts

# ======================
# 2. Helper Functions
# ======================
def generate_random_skills(num_skills=3):
    skills = {}
    for i in range(num_skills):
        skill_name = f"skill_{i}"
        proficiency = round(random.uniform(0.5, 1.0), 1)  # Higher minimum proficiency
        skills[skill_name] = proficiency
    return skills

def generate_task(possible_skills, max_req=2, max_complexity=10):
    requirements = {}
    num_req = random.randint(1, max_req)
    chosen_skills = random.sample(possible_skills, num_req)
    for skill in chosen_skills:
        requirements[skill] = round(random.uniform(0.4, 0.8), 1)  # Easier requirements
    complexity = random.randint(3, max_complexity)  # Shorter tasks
    return requirements, complexity

def build_holarchy(agents, levels=2):
    if len(agents) == 1:
        return agents[0]
    
    members = []
    group_size = max(2, len(agents) // levels)
    
    for i in range(0, len(agents), group_size):
        group = agents[i:i+group_size]
        if len(group) == 1:
            members.append(group[0])
        else:
            holon_id = f"holon_{i//group_size}"
            members.append(Holon(holon_id, group))
    
    return Holon("root", members)

# ======================
# 3. Simulation Runner
# ======================
def run_simulation(use_holonic=True, num_agents=10, num_tasks=20, failure_rate=0.1):
    print(f"Starting {'holonic' if use_holonic else 'standard'} simulation with {num_agents} agents, {num_tasks} tasks...")
    possible_skills = [f"skill_{i}" for i in range(3)]
    agents = [Agent(f"agent_{i}", generate_random_skills()) for i in range(num_agents)]
    
    # Ensure agents have sufficient skills
    for agent in agents:
        for skill, proficiency in agent.skills.items():
            if proficiency < 0.5:
                agent.skills[skill] = 0.5
    
    if use_holonic:
        root_holon = build_holarchy(agents)
    else:
        root_holon = None
    
    tasks = []
    completed_tasks = 0
    total_messages = 0
    current_time = 0
    task_queue = deque()
    
    metrics = {
        'success_rate': 0,
        'avg_completion_time': 0,
        'total_messages': 0,
        'completion_times': []
    }
    
    max_time = num_tasks * 10  # More reasonable timeout
    
    # Create all tasks upfront
    for i in range(num_tasks):
        req, comp = generate_task(possible_skills)
        task = Task(f"task_{i}", req, comp)
        task.creation_time = 0
        tasks.append(task)
        task_queue.append(task)
    
    failure_injected = False
    
    while completed_tasks < num_tasks and current_time < max_time:
        # Process task queue with a limit per timestep
        processed_tasks = []
        
        for _ in range(min(len(task_queue), 5)):  # Process up to 5 tasks per timestep
            task = task_queue.popleft()
            task.attempts += 1
            
            # Give up on tasks that can't be assigned after 5 attempts
            if task.attempts > 5:
                continue
                
            messages_before = total_messages
            assigned = False
            
            if use_holonic:
                winner, bid_val = root_holon.assign_task(task)
                if winner:
                    winner.assign_task(task)
                    total_messages += 1
                    assigned = True
            else:
                bids = []
                # Only consider available, non-failed agents
                candidate_agents = [a for a in agents if a.available and not a.failed]
                for agent in candidate_agents:
                    bid_val = agent.bid(task)
                    total_messages += 2  # Request + response
                    if bid_val is not None:
                        bids.append((agent, bid_val))
                
                if bids:
                    winner = max(bids, key=lambda x: x[1])[0]
                    # CRITICAL FIX: Ensure winner is valid before assignment
                    if winner and winner.available and not winner.failed:
                        winner.assign_task(task)
                        total_messages += 1  # Assignment message
                        assigned = True
            
            if not assigned:
                task_queue.append(task)
            else:
                processed_tasks.append(task)
        
        # Inject failures once at 25% of max_time
        if not failure_injected and current_time > max_time // 4:
            num_failures = max(1, int(num_agents * failure_rate))
            failed_agents = random.sample(agents, num_failures)
            for agent in failed_agents:
                agent.fail()
                if agent.current_task:
                    # Requeue task if agent was working on one
                    task_queue.append(agent.current_task)
                    agent.current_task = None
            failure_injected = True
            print(f"Injected {num_failures} failures at timestep {current_time}")
        
        # Agents work on tasks
        for agent in agents:
            if agent.work():
                task = agent.current_task
                task.completion_time = current_time
                metrics['completion_times'].append(current_time - task.creation_time)
                completed_tasks += 1
                agent.current_task = None
                agent.available = True
        
        # Update holon availability
        if use_holonic:
            root_holon.update_availability()
        
        current_time += 1
    
    if current_time >= max_time:
        print(f"Simulation timed out at {current_time} timesteps")
    
    # Calculate metrics
    metrics['success_rate'] = completed_tasks / num_tasks
    if metrics['completion_times']:
        metrics['avg_completion_time'] = np.mean(metrics['completion_times'])
    else:
        metrics['avg_completion_time'] = 0
    metrics['total_messages'] = total_messages
    
    print(f"Completed {completed_tasks}/{num_tasks} tasks in {current_time} timesteps")
    return metrics

# ======================
# 4. Comparison Runner
# ======================
def compare_approaches(num_agents=10, num_tasks=20, failure_rate=0.1):
    print(f"\nRunning comparison with {num_agents} agents and {num_tasks} tasks...")
    
    # Run standard agent-based
    start_time = time.time()
    std_metrics = run_simulation(
        use_holonic=False,
        num_agents=num_agents,
        num_tasks=num_tasks,
        failure_rate=failure_rate
    )
    std_time = time.time() - start_time
    
    # Run holonic
    start_time = time.time()
    holonic_metrics = run_simulation(
        use_holonic=True,
        num_agents=num_agents,
        num_tasks=num_tasks,
        failure_rate=failure_rate
    )
    holonic_time = time.time() - start_time
    
    # Print results
    print("\nResults:")
    print(f"{'Metric':<25} | {'Standard':<12} | {'Holonic':<12} | Improvement")
    print("-" * 65)
    for metric in ['success_rate', 'avg_completion_time', 'total_messages']:
        std_val = std_metrics[metric]
        holonic_val = holonic_metrics[metric]
        
        if metric == 'success_rate':
            std_str = f"{std_val:.1%}"
            holonic_str = f"{holonic_val:.1%}"
            if std_metrics[metric] > 0:
                improvement = f"{(holonic_metrics[metric] - std_metrics[metric]) / std_metrics[metric]:.0%}"
            else:
                improvement = "N/A"
            print(f"{metric:<25} | {std_str:<12} | {holonic_str:<12} | {improvement}")
        else:
            std_str = f"{std_val:.2f}"
            holonic_str = f"{holonic_val:.2f}"
            if metric == 'avg_completion_time':
                if std_metrics[metric] > 0:
                    improvement = f"{(std_metrics[metric] - holonic_metrics[metric]) / std_metrics[metric]:.0%} faster"
                else:
                    improvement = "N/A"
            else:  # total_messages
                if std_metrics[metric] > 0:
                    improvement = f"{(std_metrics[metric] - holonic_metrics[metric]) / std_metrics[metric]:.0%} less"
                else:
                    improvement = "N/A"
            print(f"{metric:<25} | {std_str:<12} | {holonic_str:<12} | {improvement}")
    
    print(f"\nSimulation times: Standard={std_time:.2f}s, Holonic={holonic_time:.2f}s")
    
    return std_metrics, holonic_metrics

# ======================
# 5. Visualization
# ======================
def plot_results(std_metrics, holonic_metrics, num_tasks=20):
    labels = ['Success Rate', 'Avg. Completion Time', 'Messages per Task']
    std_values = [
        std_metrics['success_rate'],
        std_metrics['avg_completion_time'],
        std_metrics['total_messages'] / num_tasks
    ]
    holonic_values = [
        holonic_metrics['success_rate'],
        holonic_metrics['avg_completion_time'],
        holonic_metrics['total_messages'] / num_tasks
    ]
    
    x = np.arange(len(labels))
    width = 0.35
    
    fig, ax = plt.subplots(figsize=(10, 6))
    rects1 = ax.bar(x - width/2, std_values, width, label='Standard', color='#1f77b4')
    rects2 = ax.bar(x + width/2, holonic_values, width, label='Holonic', color='#ff7f0e')
    
    ax.set_ylabel('Performance')
    ax.set_title('Agent-Based vs Holonic SoS Performance')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    
    for rect in rects1 + rects2:
        height = rect.get_height()
        ax.annotate(f'{height:.2f}' if height > 1 else f'{height:.0%}',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig('sos_comparison.png')
    plt.show()

# ======================
# 6. Run the Comparison
# ======================
if __name__ == "__main__":
    num_agents = 1000 
    num_tasks = 2000   
    
    std_metrics, holonic_metrics = compare_approaches(
        num_agents=num_agents,
        num_tasks=num_tasks,
        failure_rate=0.1
    )
    plot_results(std_metrics, holonic_metrics, num_tasks)