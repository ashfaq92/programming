import utils
from robot_cooperative_run import Simulation as CooperativeSimulation
from robot_greedy_run import Simulation as GreedySimulation
import matplotlib.pyplot as plt

def run_diagnostic():
    """Run diagnostic to compare cooperative vs greedy robots"""
    
    print("=== COOPERATIVE vs GREEDY ROBOT COMPARISON ===")
    
    # Test 1: Cooperative robots simulation
    print("\n=== TEST 1: Cooperative Robots ===")
    utils.seeded_rand.seed(42)
    coop_sim = CooperativeSimulation(1000, initial_boxes=5, num_robots=30)
    coop_results = coop_sim.run()
    
    # Test 2: Greedy robots simulation (reset seed for fair comparison)
    print("\n=== TEST 2: Greedy Robots ===")
    utils.seeded_rand.seed(42)
    greedy_sim = GreedySimulation(1000, initial_boxes=5, num_robots=30)
    greedy_results = greedy_sim.run()
    
    # Compare results
    print(f"\n{'='*50}")
    print("COMPARISON RESULTS")
    print(f"{'='*50}")
    print(f"Cooperative robots deposited: {coop_results['deposited_boxes']}")
    print(f"Greedy robots deposited: {greedy_results['deposited_boxes']}")
    print(f"Cooperative energy consumed: {coop_results['total_energy_consumed']}")
    print(f"Greedy energy consumed: {greedy_results['total_energy_consumed']}")
    
    # Generate comparison plot
    plot_comparison(coop_sim, greedy_sim)
    
    return coop_results, greedy_results

def plot_comparison(coop_sim, greedy_sim):
    """Generate comparative plots"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # Plot 1: Average Energy
    axes[0, 0].plot(coop_sim.time_series['steps'], coop_sim.time_series['avg_energy'],
                    'b-', label='Cooperative', linewidth=2)
    axes[0, 0].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['avg_energy'],
                    'r-', label='Greedy', linewidth=2)
    axes[0, 0].set_title('Average Robot Energy')
    axes[0, 0].set_xlabel('Time Steps')
    axes[0, 0].set_ylabel('Energy')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Boxes on Grid
    axes[0, 1].plot(coop_sim.time_series['steps'], coop_sim.time_series['boxes_on_grid'],
                    'b-', label='Cooperative', linewidth=2)
    axes[0, 1].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['boxes_on_grid'],
                    'r-', label='Greedy', linewidth=2)
    axes[0, 1].set_title('Boxes on Grid')
    axes[0, 1].set_xlabel('Time Steps')
    axes[0, 1].set_ylabel('Number of Boxes')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Deposited Boxes
    axes[1, 0].plot(coop_sim.time_series['steps'], coop_sim.time_series['deposited_boxes'],
                    'b-', label='Cooperative', linewidth=2)
    axes[1, 0].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['deposited_boxes'],
                    'r-', label='Greedy', linewidth=2)
    axes[1, 0].set_title('Cumulative Deposited Boxes')
    axes[1, 0].set_xlabel('Time Steps')
    axes[1, 0].set_ylabel('Boxes Deposited')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: Alive Robots
    axes[1, 1].plot(coop_sim.time_series['steps'], coop_sim.time_series['alive_robots'],
                    'b-', label='Cooperative', linewidth=2)
    axes[1, 1].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['alive_robots'],
                    'r-', label='Greedy', linewidth=2)
    axes[1, 1].set_title('Alive Robots')
    axes[1, 1].set_xlabel('Time Steps')
    axes[1, 1].set_ylabel('Number of Robots')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Save plot
    plt.savefig('cooperative_vs_greedy_comparison.png', dpi=300, bbox_inches='tight')
    print("Comparison plot saved as 'cooperative_vs_greedy_comparison.png'")

if __name__ == "__main__":
    run_diagnostic()