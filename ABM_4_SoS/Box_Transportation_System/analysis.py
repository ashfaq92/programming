import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
from environment.grid import Grid
from robot import Robot
import utils


def run_aggressive_analysis():
    """
    AGGRESSIVE parameter tuning to force realistic robot deaths
    """

    # Store original parameters
    original_energy_cost = utils.ENERGY_COST
    original_reward = utils.REWARD_AMOUNT
    original_bonus = utils.BONUS_AMOUNT

    # MUCH MORE AGGRESSIVE PARAMETERS
    utils.ENERGY_COST = 2.0  # Double the movement cost
    utils.REWARD_AMOUNT = 2.0  # Reduce reward
    utils.BONUS_AMOUNT = 0.0  # Reduce bonus
    # Net: 3.0 energy per deposit, 2.0 cost per move
    # With 31.7 moves per deposit: 63.4 cost - 3.0 reward = -60.4 energy loss per successful deposit!

    print("AGGRESSIVE PARAMETERS:")
    print(f"  Energy cost per move: {utils.ENERGY_COST}")
    print(f"  Energy gained per deposit: {utils.REWARD_AMOUNT + utils.BONUS_AMOUNT}")
    print(f"  Break-even moves per deposit: {(utils.REWARD_AMOUNT + utils.BONUS_AMOUNT) / utils.ENERGY_COST}")
    print(
        f"  Expected energy loss per deposit: {31.7 * utils.ENERGY_COST - (utils.REWARD_AMOUNT + utils.BONUS_AMOUNT):.1f}")
    print(
        f"  Expected robot lifespan: ~{350 / (31.7 * utils.ENERGY_COST - (utils.REWARD_AMOUNT + utils.BONUS_AMOUNT)):.1f} deposits")

    # Initialize
    grid = Grid(width=50, height=50)

    # Create 35 robots with LOWER starting energy for faster deaths
    STARTING_ENERGY = 250  # Reduced from 350
    TOTAL_ROBOTS = 35

    robots = []
    robots_per_color = TOTAL_ROBOTS // len(utils.COLORS)
    remainder = TOTAL_ROBOTS % len(utils.COLORS)

    for i, color in enumerate(utils.COLORS):
        robot_count = robots_per_color + (1 if i < remainder else 0)
        for _ in range(robot_count):
            robot = Robot(e=STARTING_ENERGY, c=color, grid=grid)
            robots.append(robot)
            grid.add_robot(robot)

    grid.initialize_boxes(15)  # Fewer initial boxes

    print(f"\nCreated {len(robots)} robots with {STARTING_ENERGY} starting energy")
    print(f"Grid has {len(grid.nests)} nests")

    # Tracking
    metrics = {
        'cycle': [],
        'mean_battery': [],
        'total_boxes': [],
        'alive_robots': [],
        'boxes_deposited': [],
        'active_robots': [],
        'deaths_this_period': []
    }

    print("\nRunning AGGRESSIVE simulation...")
    print("=" * 60)

    deaths_total = 0

    for cycle in range(10000):
        # More conservative box generation
        if cycle % 25 == 0 and len(grid.boxes) < 100:  # Less frequent, lower cap
            boxes_to_add = min(3, 100 - len(grid.boxes))
            grid.initialize_boxes(boxes_to_add)

        # Count deaths this cycle
        alive_before = sum(1 for robot in grid.robots if robot.energy > 0)

        # Robots act
        for robot in grid.robots:
            if robot.energy > 0:
                try:
                    robot.step()
                except:
                    continue

        # Count deaths after
        alive_after = sum(1 for robot in grid.robots if robot.energy > 0)
        deaths_this_cycle = alive_before - alive_after
        deaths_total += deaths_this_cycle

        # Record metrics every 10 cycles
        if cycle % 10 == 0:
            robot_energies = [robot.energy for robot in grid.robots]
            alive_energies = [e for e in robot_energies if e > 0]

            if alive_energies:
                mean_battery = np.mean(alive_energies)
            else:
                mean_battery = 0

            alive_robots = len(alive_energies)
            active_robots = sum(1 for energy in robot_energies if energy > utils.ENERGY_COST)
            total_boxes = len(grid.boxes)
            boxes_deposited = sum(len(nest.deposited_boxes) for nest in grid.nests)

            metrics['cycle'].append(cycle)
            metrics['mean_battery'].append(mean_battery)
            metrics['total_boxes'].append(total_boxes)
            metrics['alive_robots'].append(alive_robots)
            metrics['boxes_deposited'].append(boxes_deposited)
            metrics['active_robots'].append(active_robots)
            metrics['deaths_this_period'].append(deaths_this_cycle)

        # Enhanced progress reporting
        if cycle % 500 == 0:
            alive_count = sum(1 for robot in grid.robots if robot.energy > 0)
            boxes_deposited = sum(len(nest.deposited_boxes) for nest in grid.nests)
            mean_battery = np.mean(
                [robot.energy for robot in grid.robots if robot.energy > 0]) if alive_count > 0 else 0

            print(f"Cycle {cycle:5d}: {alive_count:2d} alive ({deaths_total:2d} died), "
                  f"{len(grid.boxes):3d} boxes, {boxes_deposited:3d} deposited, battery: {mean_battery:6.1f}")

        # Stop if too few robots
        if alive_after < 3:
            print(f"Only {alive_after} robots remaining at cycle {cycle}, ending simulation")
            break

    # Restore parameters
    utils.ENERGY_COST = original_energy_cost
    utils.REWARD_AMOUNT = original_reward
    utils.BONUS_AMOUNT = original_bonus

    # Final stats
    final_alive = sum(1 for robot in grid.robots if robot.energy > 0)
    survival_rate = (final_alive / TOTAL_ROBOTS) * 100

    print(f"\n" + "=" * 60)
    print("AGGRESSIVE RESULTS:")
    print(f"Robots died: {TOTAL_ROBOTS - final_alive}/{TOTAL_ROBOTS} ({100 - survival_rate:.1f}%)")
    print(f"Survivors: {final_alive} ({survival_rate:.1f}%)")
    print(f"Total boxes deposited: {sum(len(nest.deposited_boxes) for nest in grid.nests)}")

    return metrics, grid


def create_aggressive_plots(metrics):
    """Create plots for aggressive analysis"""

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('CoCaRo AGGRESSIVE Analysis - Forcing Robot Deaths', fontsize=16, fontweight='bold')

    # Plot 1: Mean Battery
    axes[0, 0].plot(metrics['cycle'], metrics['mean_battery'], 'b-', linewidth=2)
    axes[0, 0].set_title('Mean Battery Level')
    axes[0, 0].set_xlabel('Cycle')
    axes[0, 0].set_ylabel('Energy')
    axes[0, 0].axhspan(50, 100, alpha=0.2, color='green', label='Target Range')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Plot 2: Robot Population with Deaths
    axes[0, 1].plot(metrics['cycle'], metrics['alive_robots'], 'r-', linewidth=2, label='Alive')
    axes[0, 1].plot(metrics['cycle'], metrics['active_robots'], 'orange', linewidth=2, label='Active')
    axes[0, 1].set_title('Robot Population')
    axes[0, 1].set_xlabel('Cycle')
    axes[0, 1].set_ylabel('Count')
    axes[0, 1].axhspan(8, 19, alpha=0.2, color='green', label='Target Final Range')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Plot 3: Boxes
    axes[0, 2].plot(metrics['cycle'], metrics['total_boxes'], 'g-', linewidth=2)
    axes[0, 2].set_title('Boxes on Grid')
    axes[0, 2].set_xlabel('Cycle')
    axes[0, 2].set_ylabel('Count')
    axes[0, 2].grid(True, alpha=0.3)

    # Plot 4: Deaths over time
    deaths_cumulative = np.cumsum(metrics['deaths_this_period'])
    axes[1, 0].plot(metrics['cycle'], deaths_cumulative, 'black', linewidth=2)
    axes[1, 0].set_title('Cumulative Robot Deaths')
    axes[1, 0].set_xlabel('Cycle')
    axes[1, 0].set_ylabel('Deaths')
    axes[1, 0].grid(True, alpha=0.3)

    # Plot 5: Boxes Deposited
    axes[1, 1].plot(metrics['cycle'], metrics['boxes_deposited'], 'purple', linewidth=2)
    axes[1, 1].set_title('Boxes Deposited')
    axes[1, 1].set_xlabel('Cycle')
    axes[1, 1].set_ylabel('Count')
    axes[1, 1].grid(True, alpha=0.3)

    # Plot 6: System Efficiency
    efficiency = []
    for i in range(len(metrics['cycle'])):
        if metrics['alive_robots'][i] > 0 and metrics['cycle'][i] > 0:
            eff = metrics['boxes_deposited'][i] / metrics['alive_robots'][i] / (metrics['cycle'][i] / 1000)
        else:
            eff = 0
        efficiency.append(eff)

    axes[1, 2].plot(metrics['cycle'], efficiency, 'brown', linewidth=2)
    axes[1, 2].set_title('Deposition Efficiency')
    axes[1, 2].set_xlabel('Cycle')
    axes[1, 2].set_ylabel('Boxes per Robot per 1000 cycles')
    axes[1, 2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('cocaro_aggressive_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    print("Running AGGRESSIVE CoCaRo Analysis...")
    print("This should finally kill some robots! 💀")

    metrics, final_grid = run_aggressive_analysis()
    create_aggressive_plots(metrics)

    print("\nIf robots STILL don't die, we need to check the robot.step() implementation!")