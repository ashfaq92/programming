from Box_Transportation_System import utils
from environment.grid import Grid
from robot import Robot
import matplotlib.pyplot as plt
import time
import random

def single_cocaro_simulation(seed=None, verbose=False):
    """Run a single CoCaRo simulation and return results"""
    
    # Set seed for this simulation
    if seed is not None:
        utils.RAND_SEED = seed
        utils.seeded_rand = random.Random(seed)
    
    if verbose:
        print(f"Running simulation with seed: {utils.RAND_SEED}")
    
    # Create grid
    grid = Grid(width=50, height=50)
    
    # Create 90 robots (30 per color)
    robots = []
    for color in utils.COLORS:
        for _ in range(30):
            robot = Robot(e=utils.INITIAL_ENERGY, c=color, grid=grid)
            robots.append(robot)
            grid.add_robot(robot)

    # Initialize some boxes
    grid.initialize_boxes(10)
    
    # Data collection
    steps = []
    alive_counts = []
    box_counts = []
    avg_energies = []
    
    # Run simulation
    CYCLES = 10000
    
    for step in range(CYCLES):
        # Add 1 box every 3 time units
        if step % 3 == 0:
            grid.initialize_boxes(1)
        
        # Each robot acts
        for robot in robots:
            robot.step()
        
        # Remove dead robots from grid
        for y in range(grid.height):
            for x in range(grid.width):
                cell = grid.cells[y][x]
                if cell.robot and cell.robot.energy <= 0:
                    cell.robot = None
        
        # Filter alive robots
        robots = [r for r in robots if r.energy > 0]
        
        # Collect data
        steps.append(step)
        alive_counts.append(len(robots))
        box_counts.append(len(grid.boxes))
        
        if robots:
            avg_energy = sum(r.energy for r in robots) / len(robots)
        else:
            avg_energy = 0
        avg_energies.append(avg_energy)
        
        # Print progress (only if verbose)
        if verbose and step % 1000 == 0:
            boxes_deposited = sum(len(nest.deposited_boxes) for nest in grid.nests)
            print(f"  Step {step:4d}: {len(robots):2d} alive, {len(grid.boxes):3d} boxes, "
                  f"{boxes_deposited:3d} deposited, avg energy: {avg_energy:.1f}")
        
        # Stop if all robots are dead
        if not robots:
            if verbose:
                print(f"  All robots died at step {step}")
            break
    
    # Calculate final results
    total_deposited = sum(len(nest.deposited_boxes) for nest in grid.nests)
    survival_time = len(steps) - 1
    final_boxes = len(grid.boxes) if len(steps) > 0 else 0
    
    # Return results dictionary
    return {
        'seed': utils.RAND_SEED,
        'survivors': len(robots),
        'total_deposited': total_deposited,
        'survival_time': survival_time,
        'final_boxes': final_boxes,
        'steps': steps,
        'alive_counts': alive_counts,
        'box_counts': box_counts,
        'avg_energies': avg_energies,
        'nest_deposits': {nest.color: len(nest.deposited_boxes) for nest in grid.nests}
    }

def run_multiple_simulations(num_runs=5):
    """Run multiple simulations with different random seeds"""
    
    print("=== CoCaRo Multiple Simulation Analysis ===")
    print(f"Energy cost: {utils.ENERGY_COST}, Reward: {utils.REWARD_AMOUNT}, Bonus: {utils.BONUS_AMOUNT}")
    print(f"90 robots, perception radius: {utils.PERCEPTION_RADIUS}")
    print(f"Running {num_runs} simulations with different seeds...\n")
    
    all_results = []
    
    for run in range(num_runs):
        print(f"=== SIMULATION RUN {run + 1}/{num_runs} ===")
        
        # Use different seed for each run
        seed = run + int(time.time() * 1000) % 10000  # Ensure different seeds
        
        # Run single simulation
        result = single_cocaro_simulation(seed=seed, verbose=True)
        all_results.append(result)
        
        # Print run summary
        print(f"Run {run + 1} Results:")
        print(f"  Seed: {result['seed']}")
        print(f"  Survivors: {result['survivors']}/90")
        print(f"  Total deposited: {result['total_deposited']}")
        print(f"  Survival time: {result['survival_time']} steps")
        print(f"  Final boxes: {result['final_boxes']}")
        
        # Print nest-specific deposits
        for color, count in result['nest_deposits'].items():
            print(f"  {color} nest: {count} boxes")
        print()
    
    # Calculate statistics across all runs
    print("=== STATISTICAL ANALYSIS ACROSS ALL RUNS ===")
    
    survivors = [r['survivors'] for r in all_results]
    deposited = [r['total_deposited'] for r in all_results]
    survival_times = [r['survival_time'] for r in all_results]
    final_boxes = [r['final_boxes'] for r in all_results]
    
    print(f"Survivors: Mean={sum(survivors)/num_runs:.1f}, Min={min(survivors)}, Max={max(survivors)}")
    print(f"Deposited: Mean={sum(deposited)/num_runs:.1f}, Min={min(deposited)}, Max={max(deposited)}")
    print(f"Survival time: Mean={sum(survival_times)/num_runs:.1f}, Min={min(survival_times)}, Max={max(survival_times)}")
    print(f"Final boxes: Mean={sum(final_boxes)/num_runs:.1f}, Min={min(final_boxes)}, Max={max(final_boxes)}")
    
    # Create comparative plots
    create_multiple_run_plots(all_results, num_runs)
    
    return all_results

def create_multiple_run_plots(all_results, num_runs):
    """Create plots comparing multiple simulation runs"""
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 12))
    fig.suptitle(f'CoCaRo Multiple Simulation Comparison ({num_runs} runs)', fontsize=16, fontweight='bold')
    
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    
    for i, result in enumerate(all_results):
        color = colors[i % len(colors)]
        alpha = 0.7
        label = f"Run {i+1} (seed {result['seed']})"
        
        # Plot 1: Robot population
        ax1.plot(result['steps'], result['alive_counts'], 
                color=color, alpha=alpha, linewidth=1.5, label=label)
        
        # Plot 2: Box counts
        ax2.plot(result['steps'], result['box_counts'], 
                color=color, alpha=alpha, linewidth=1.5, label=label)
        
        # Plot 3: Average energies
        ax3.plot(result['steps'], result['avg_energies'], 
                color=color, alpha=alpha, linewidth=1.5, label=label)
    
    # Configure plots
    ax1.set_ylabel('Number of Alive Robots', fontsize=12)
    ax1.set_title('Robot Population Over Time (Multiple Runs)', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 100)
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    ax2.set_ylabel('Number of Boxes', fontsize=12)
    ax2.set_title('Box Accumulation Over Time (Multiple Runs)', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    ax3.set_ylabel('Average Energy Level', fontsize=12)
    ax3.set_xlabel('Simulation Steps', fontsize=12)
    ax3.set_title('Mean Battery Level Over Time (Multiple Runs)', fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 320)
    ax3.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Adjust layout and save
    plt.tight_layout()
    plt.subplots_adjust(right=0.75)  # Make room for legend
    plt.savefig('cocaro_multiple_runs.png', dpi=300, bbox_inches='tight')
    print(f"Multiple run plot saved as 'cocaro_multiple_runs.png'")
    plt.show()

def single_simulation_with_plots():
    """Run a single simulation and show detailed plots"""
    
    print("=== Single CoCaRo Simulation with Detailed Plots ===")
    seed = int(time.time()) % 10000
    result = single_cocaro_simulation(seed=seed, verbose=True)
    
    # Create single simulation plots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f'CoCaRo Simulation Analysis (Seed: {result["seed"]})', fontsize=16, fontweight='bold')
    
    ax1.plot(result['steps'], result['alive_counts'], 'r-', linewidth=2)
    ax1.set_ylabel('Number of Alive Robots', fontsize=12)
    ax1.set_title('Robot Population Over Time', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 100)
    
    ax2.plot(result['steps'], result['box_counts'], 'g-', linewidth=2)
    ax2.set_ylabel('Number of Boxes', fontsize=12)
    ax2.set_title('Box Accumulation Over Time', fontsize=14)
    ax2.grid(True, alpha=0.3)
    
    ax3.plot(result['steps'], result['avg_energies'], 'b-', linewidth=2)
    ax3.set_ylabel('Average Energy Level', fontsize=12)
    ax3.set_xlabel('Simulation Steps', fontsize=12)
    ax3.set_title('Mean Battery Level Over Time', fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 320)
    
    plt.tight_layout()
    plt.savefig(f'cocaro_single_run_seed_{result["seed"]}.png', dpi=300, bbox_inches='tight')
    print(f"Single run plot saved as 'cocaro_single_run_seed_{result['seed']}.png'")
    plt.show()

if __name__ == "__main__":
    # Choose what to run:
    
    # Option 1: Run multiple simulations for statistical analysis
    run_multiple_simulations(num_runs=50)
    
    # Option 2: Run single simulation with detailed plots
    # single_simulation_with_plots()