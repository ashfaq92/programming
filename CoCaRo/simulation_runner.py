import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from robot_base import RobotSimulation, RobotType


def run_experiment(robot_type: str, steps: int = 1000, seed: int = 42) -> pd.DataFrame:
    """Run a single experiment with specified robot type"""
    print(f"Running {robot_type} experiment...")

    # Set random seed for reproducibility
    np.random.seed(seed)

    # Create and run model
    model = RobotSimulation(robot_type=robot_type)

    for step in range(steps):
        model.step()
        if not model.running:
            print(f"  Simulation ended early at step {step}")
            break

    # Get collected data
    data = model.datacollector.get_model_vars_dataframe()
    data['Robot_Type'] = robot_type
    data['Step'] = data.index

    return data


def compare_robot_types(steps: int = 1000, seed: int = 42):
    """Compare all robot types and create plots"""

    robot_types = ["random", "greedy", "cooperative", "saphesia"]
    all_data = []

    # Run experiments for each robot type
    for robot_type in robot_types:
        data = run_experiment(robot_type, steps, seed)
        all_data.append(data)

    # Combine all data
    combined_data = pd.concat(all_data, ignore_index=True)

    # Create comparison plots
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(f'Robot Strategy Comparison (Seed: {seed})', fontsize=16)

    # Plot 1: Mean Battery Level
    axes[0, 0].set_title('Mean Battery Level Over Time')
    for robot_type in robot_types:
        data = combined_data[combined_data['Robot_Type'] == robot_type]
        axes[0, 0].plot(data['Step'], data['Mean_Battery'], label=robot_type, linewidth=2)
    axes[0, 0].set_xlabel('Step')
    axes[0, 0].set_ylabel('Mean Battery')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Plot 2: Alive Robots
    axes[0, 1].set_title('Alive Robots Over Time')
    for robot_type in robot_types:
        data = combined_data[combined_data['Robot_Type'] == robot_type]
        axes[0, 1].plot(data['Step'], data['Alive_Robots'], label=robot_type, linewidth=2)
    axes[0, 1].set_xlabel('Step')
    axes[0, 1].set_ylabel('Alive Robots')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Plot 3: Boxes in Environment
    axes[1, 0].set_title('Boxes in Environment Over Time')
    for robot_type in robot_types:
        data = combined_data[combined_data['Robot_Type'] == robot_type]
        axes[1, 0].plot(data['Step'], data['Total_Boxes'], label=robot_type, linewidth=2)
    axes[1, 0].set_xlabel('Step')
    axes[1, 0].set_ylabel('Total Boxes')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Plot 4: Total Delivered Boxes
    axes[1, 1].set_title('Total Delivered Boxes Over Time')
    for robot_type in robot_types:
        data = combined_data[combined_data['Robot_Type'] == robot_type]
        total_delivered = data['Red_Delivered'] + data['Green_Delivered'] + data['Blue_Delivered']
        axes[1, 1].plot(data['Step'], total_delivered, label=robot_type, linewidth=2)
    axes[1, 1].set_xlabel('Step')
    axes[1, 1].set_ylabel('Total Delivered')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Print final statistics
    print("\n=== FINAL RESULTS ===")
    for robot_type in robot_types:
        data = combined_data[combined_data['Robot_Type'] == robot_type]
        final_data = data.iloc[-1]

        print(f"\n{robot_type.upper()}:")
        print(f"  Alive Robots: {final_data['Alive_Robots']}/90")
        print(f"  Mean Battery: {final_data['Mean_Battery']:.1f}")
        print(
            f"  Boxes Delivered: {final_data['Red_Delivered'] + final_data['Green_Delivered'] + final_data['Blue_Delivered']}")
        print(f"  Boxes in Environment: {final_data['Total_Boxes']}")

    return combined_data


def test_saphesia_cooperation():
    """Test SApHESIA cooperation mechanism specifically"""
    print("\n=== TESTING SAPHESIA COOPERATION ===")

    model = RobotSimulation(robot_type="saphesia")

    # Run for a few steps and check cooperation
    for step in range(50):
        model.step()

        # Check if any cooperation is happening
        if step % 10 == 0:
            cooperation_active = False
            for robot in model.robots:
                if robot.helped_colors:
                    cooperation_active = True
                    break

            if cooperation_active:
                print(f"Step {step}: Inter-group cooperation is ACTIVE!")

                # Show which robots are helping whom
                for robot in model.robots:
                    if robot.helped_colors:
                        print(f"  {robot.color} robot helping: {robot.helped_colors}")
                break
            else:
                print(f"Step {step}: No cooperation yet...")

    return model


if __name__ == "__main__":
    # Compare all robot types
    results = compare_robot_types(steps=1000, seed=42)

    # Test SApHESIA cooperation mechanism
    test_saphesia_cooperation()

    # Save results to CSV for further analysis
    results.to_csv('robot_comparison_results.csv', index=False)
    print("\nResults saved to 'robot_comparison_results.csv'")

    print("\n=== EXPERIMENT COMPLETE ===")
    print("If SApHESIA is working correctly, you should see:")
    print("1. Different performance curves for each robot type")
    print("2. Cooperation messages in the SApHESIA test")
    print("3. Potentially better performance for SApHESIA vs cooperative")