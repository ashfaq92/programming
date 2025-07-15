import random
import numpy as np
import matplotlib.pyplot as plt

from robot_base import RobotSimulation


def compare_greedy_vs_cooperative(steps=1000, seed=42):
    """Direct comparison of greedy vs cooperative strategies"""

    strategies = ["greedy", "cooperative"]
    results = {}

    for strategy in strategies:
        print(f"\n=== Running {strategy.upper()} strategy ===")

        # Set seed for reproducibility
        random.seed(seed)
        np.random.seed(seed)

        # Create model
        model = RobotSimulation(robot_type=strategy)

        # Run simulation
        for step in range(steps):
            model.step()
            if not model.running:
                print(f"  Simulation ended early at step {step}")
                break

            # Log cooperation events for cooperative strategy
            if strategy == "cooperative" and step % 100 == 0:
                cooperation_count = 0
                for robot in model.robots:
                    if hasattr(robot, 'pending_requests') and len(robot.pending_requests) > 0:
                        cooperation_count += 1
                if cooperation_count > 0:
                    print(f"  Step {step}: {cooperation_count} robots involved in cooperation")

        # Get final results
        data = model.datacollector.get_model_vars_dataframe()
        results[strategy] = data

        # Print summary
        final_alive = data['Alive_Robots'].iloc[-1]
        final_battery = data['Mean_Battery'].iloc[-1]
        total_delivered = (data['Red_Delivered'].iloc[-1] +
                           data['Green_Delivered'].iloc[-1] +
                           data['Blue_Delivered'].iloc[-1])

        print(f"  Final Results:")
        print(f"    Alive Robots: {final_alive}/90")
        print(f"    Mean Battery: {final_battery:.1f}")
        print(f"    Total Delivered: {total_delivered}")
        print(f"    Efficiency: {total_delivered / steps:.3f} boxes/step")

    # Create comparison plot
    plt.figure(figsize=(15, 10))

    # Plot 1: Alive Robots
    plt.subplot(2, 2, 1)
    for strategy in strategies:
        data = results[strategy]
        plt.plot(data.index, data['Alive_Robots'], label=strategy, linewidth=2)
    plt.title('Alive Robots Over Time')
    plt.xlabel('Step')
    plt.ylabel('Alive Robots')
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot 2: Mean Battery
    plt.subplot(2, 2, 2)
    for strategy in strategies:
        data = results[strategy]
        plt.plot(data.index, data['Mean_Battery'], label=strategy, linewidth=2)
    plt.title('Mean Battery Level Over Time')
    plt.xlabel('Step')
    plt.ylabel('Mean Battery')
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot 3: Total Delivered
    plt.subplot(2, 2, 3)
    for strategy in strategies:
        data = results[strategy]
        total_delivered = data['Red_Delivered'] + data['Green_Delivered'] + data['Blue_Delivered']
        plt.plot(data.index, total_delivered, label=strategy, linewidth=2)
    plt.title('Total Boxes Delivered Over Time')
    plt.xlabel('Step')
    plt.ylabel('Boxes Delivered')
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot 4: Boxes in Environment
    plt.subplot(2, 2, 4)
    for strategy in strategies:
        data = results[strategy]
        plt.plot(data.index, data['Total_Boxes'], label=strategy, linewidth=2)
    plt.title('Boxes in Environment Over Time')
    plt.xlabel('Step')
    plt.ylabel('Total Boxes')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.suptitle('Greedy vs Cooperative Strategy Comparison', fontsize=16, y=1.02)
    plt.show()

    return results


# Run the comparison
if __name__ == "__main__":
    results = compare_greedy_vs_cooperative(steps=1000, seed=42)

    print("\n=== FINAL COMPARISON ===")
    for strategy, data in results.items():
        final_data = data.iloc[-1]
        print(f"\n{strategy.upper()}:")
        print(f"  Survival Rate: {final_data['Alive_Robots']}/90 ({final_data['Alive_Robots'] / 90 * 100:.1f}%)")
        print(f"  Mean Battery: {final_data['Mean_Battery']:.1f}")
        print(
            f"  Total Delivered: {final_data['Red_Delivered'] + final_data['Green_Delivered'] + final_data['Blue_Delivered']}")