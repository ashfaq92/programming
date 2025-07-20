import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from faithful_gama_translation import BoxTransportModel, RobotWithoutBuffer, DoubleCooprRobotNobuff, Box
import time
from dataclasses import dataclass
from typing import List

@dataclass
class SimulationResults:
    step: List[int]
    mean_battery: List[float]
    alive_robots: List[int]
    boxes_in_environment: List[int]
    boxes_delivered: List[int]
    model_name: str

def run_simulation(cooperation_type: str, steps: int = 500, seed: int = None) -> SimulationResults:
    """Run a single simulation"""
    if seed is not None:
        import random
        random.seed(seed)
        np.random.seed(seed)
    
    model = BoxTransportModel(width=20, height=20, cooperation_type=cooperation_type)
    
    # Data collection
    step_data = []
    mean_battery_data = []
    alive_robots_data = []
    boxes_in_env_data = []
    boxes_delivered_data = []
    
    for step in range(steps):
        model.step()
        
        # Get alive robots (check both types properly)
        alive_robots = [agent for agent in model.schedule.agents 
                       if hasattr(agent, 'battery') and hasattr(agent, 'is_dead') and not agent.is_dead]
        
        # Get active boxes
        active_boxes = [agent for agent in model.schedule.agents 
                       if isinstance(agent, Box) and not agent.this_is_the_end]
        
        # Calculate delivered boxes
        total_boxes_created = model.box_counter
        boxes_delivered = total_boxes_created - len(active_boxes)
        
        # Calculate metrics
        step_data.append(step + 1)
        
        if alive_robots:
            mean_battery = np.mean([robot.battery for robot in alive_robots])
        else:
            mean_battery = 0
        
        mean_battery_data.append(mean_battery)
        alive_robots_data.append(len(alive_robots))
        boxes_in_env_data.append(len(active_boxes))
        boxes_delivered_data.append(boxes_delivered)
    
    model_name = "Basic Cooperation" if cooperation_type == "basic" else "Enhanced Cooperation"
    
    return SimulationResults(
        step=step_data,
        mean_battery=mean_battery_data,
        alive_robots=alive_robots_data,
        boxes_in_environment=boxes_in_env_data,
        boxes_delivered=boxes_delivered_data,
        model_name=model_name
    )

def run_multiple_simulations(n_runs: int = 3, steps: int = 500) -> tuple:
    """Run multiple simulations for both models"""
    print(f"🔄 Running {n_runs} simulations for each model...")
    
    basic_results = []
    enhanced_results = []
    
    for run in range(n_runs):
        print(f"   Run {run + 1}/{n_runs}")
        
        seed = run * 1000
        
        # Run basic model
        basic_result = run_simulation("basic", steps, seed)
        basic_results.append(basic_result)
        
        # Run enhanced model
        enhanced_result = run_simulation("enhanced", steps, seed)
        enhanced_results.append(enhanced_result)
    
    return basic_results, enhanced_results

def create_comparison_plots(basic_results: List[SimulationResults], 
                          enhanced_results: List[SimulationResults]):
    """Create comparison plots"""
    
    # Average results
    basic_avg = aggregate_results(basic_results)
    enhanced_avg = aggregate_results(enhanced_results)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Faithful GAMA Translation: Robot Cooperation Comparison', 
                 fontsize=16, fontweight='bold')
    
    # Plot 1: Mean Battery Level
    ax1 = axes[0, 0]
    ax1.plot(basic_avg.step, basic_avg.mean_battery, 'b-', linewidth=2, 
             label='Basic Cooperation', alpha=0.8)
    ax1.plot(enhanced_avg.step, enhanced_avg.mean_battery, 'r-', linewidth=2, 
             label='Enhanced Cooperation', alpha=0.8)
    ax1.set_xlabel('Time Steps')
    ax1.set_ylabel('Mean Battery Level')
    ax1.set_title('Battery Level Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Number of Alive Robots
    ax2 = axes[0, 1]
    ax2.plot(basic_avg.step, basic_avg.alive_robots, 'b-', linewidth=2, 
             label='Basic Cooperation', alpha=0.8)
    ax2.plot(enhanced_avg.step, enhanced_avg.alive_robots, 'r-', linewidth=2, 
             label='Enhanced Cooperation', alpha=0.8)
    ax2.set_xlabel('Time Steps')
    ax2.set_ylabel('Number of Alive Robots')
    ax2.set_title('Robot Survival Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Boxes in Environment
    ax3 = axes[1, 0]
    ax3.plot(basic_avg.step, basic_avg.boxes_in_environment, 'b-', linewidth=2, 
             label='Basic Cooperation', alpha=0.8)
    ax3.plot(enhanced_avg.step, enhanced_avg.boxes_in_environment, 'r-', linewidth=2, 
             label='Enhanced Cooperation', alpha=0.8)
    ax3.set_xlabel('Time Steps')
    ax3.set_ylabel('Boxes in Environment')
    ax3.set_title('Remaining Boxes Over Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Boxes Delivered (Cumulative)
    ax4 = axes[1, 1]
    ax4.plot(basic_avg.step, basic_avg.boxes_delivered, 'b-', linewidth=2, 
             label='Basic Cooperation', alpha=0.8)
    ax4.plot(enhanced_avg.step, enhanced_avg.boxes_delivered, 'r-', linewidth=2, 
             label='Enhanced Cooperation', alpha=0.8)
    ax4.set_xlabel('Time Steps')
    ax4.set_ylabel('Boxes Delivered (Cumulative)')
    ax4.set_title('Box Delivery Performance')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('faithful_gama_comparison.png', dpi=300, bbox_inches='tight')
    print("📊 Plots saved as 'faithful_gama_comparison.png'")
    plt.show()

def aggregate_results(results: List[SimulationResults]) -> SimulationResults:
    """Aggregate multiple simulation results"""
    if not results:
        return None
    
    # Calculate means across runs
    mean_battery_avg = np.mean([result.mean_battery for result in results], axis=0)
    alive_robots_avg = np.mean([result.alive_robots for result in results], axis=0)
    boxes_in_env_avg = np.mean([result.boxes_in_environment for result in results], axis=0)
    boxes_delivered_avg = np.mean([result.boxes_delivered for result in results], axis=0)
    
    return SimulationResults(
        step=results[0].step,
        mean_battery=mean_battery_avg.tolist(),
        alive_robots=alive_robots_avg.tolist(),
        boxes_in_environment=boxes_in_env_avg.tolist(),
        boxes_delivered=boxes_delivered_avg.tolist(),
        model_name=results[0].model_name
    )

def print_summary_statistics(basic_results: List[SimulationResults], 
                           enhanced_results: List[SimulationResults]):
    """Print summary statistics"""
    
    print("\n" + "="*60)
    print("📊 FAITHFUL GAMA TRANSLATION RESULTS")
    print("="*60)
    
    # Final metrics
    basic_final_batteries = [result.mean_battery[-1] for result in basic_results]
    basic_final_alive = [result.alive_robots[-1] for result in basic_results]
    basic_total_delivered = [result.boxes_delivered[-1] for result in basic_results]
    
    enhanced_final_batteries = [result.mean_battery[-1] for result in enhanced_results]
    enhanced_final_alive = [result.alive_robots[-1] for result in enhanced_results]
    enhanced_total_delivered = [result.boxes_delivered[-1] for result in enhanced_results]
    
    print(f"\n🔋 FINAL BATTERY LEVELS:")
    print(f"   Basic Cooperation:    {np.mean(basic_final_batteries):.1f} ± {np.std(basic_final_batteries):.1f}")
    print(f"   Enhanced Cooperation: {np.mean(enhanced_final_batteries):.1f} ± {np.std(enhanced_final_batteries):.1f}")
    
    print(f"\n🤖 ROBOTS SURVIVED:")
    print(f"   Basic Cooperation:    {np.mean(basic_final_alive):.1f} ± {np.std(basic_final_alive):.1f}")
    print(f"   Enhanced Cooperation: {np.mean(enhanced_final_alive):.1f} ± {np.std(enhanced_final_alive):.1f}")
    
    print(f"\n📦 BOXES DELIVERED:")
    print(f"   Basic Cooperation:    {np.mean(basic_total_delivered):.1f} ± {np.std(basic_total_delivered):.1f}")
    print(f"   Enhanced Cooperation: {np.mean(enhanced_total_delivered):.1f} ± {np.std(enhanced_total_delivered):.1f}")
    
    # Performance improvements
    battery_improvement = ((np.mean(enhanced_final_batteries) - np.mean(basic_final_batteries)) / np.mean(basic_final_batteries)) * 100
    survival_improvement = ((np.mean(enhanced_final_alive) - np.mean(basic_final_alive)) / np.mean(basic_final_alive)) * 100
    delivery_improvement = ((np.mean(enhanced_total_delivered) - np.mean(basic_total_delivered)) / max(np.mean(basic_total_delivered), 1)) * 100
    
    print(f"\n📈 PERFORMANCE IMPROVEMENTS (Enhanced vs Basic):")
    print(f"   Battery Level:        {battery_improvement:+.1f}%")
    print(f"   Robot Survival:       {survival_improvement:+.1f}%")
    print(f"   Box Delivery:         {delivery_improvement:+.1f}%")
    
    print("\n" + "="*60)

def main():
    """Main comparison using faithful GAMA translation"""
    print("🚀 FAITHFUL GAMA TRANSLATION COMPARISON - CONSERVATIVE COOPERATION")
    print("="*68)
    print("🔧 MAJOR BUG FIXES APPLIED:")
    print("   • Fixed box transfer logic (robots were losing boxes incorrectly)")
    print("   • Made cooperation more conservative (+20 criticality threshold)")
    print("   • Reduced negotiation overhead (only when desperate)")
    print("   • Better reservation system (only transfer if actually agreed)")
    print("   • Focus on free boxes first, minimal negotiation")
    print()
    
    # Parameters - same stress test
    n_runs = 2
    steps = 2000
    
    print(f"📋 CONSERVATIVE COOPERATION TEST:")
    print(f"   • 90 robots (30 per color)")
    print(f"   • 20x20 grid")
    print(f"   • {steps} time steps")
    print(f"   • {n_runs} runs per model")
    print(f"   • Box generation: 1 every 3 steps")
    print(f"   • Expected: Cooperation helps survival WITHOUT hurting efficiency")
    
    # Run simulations
    start_time = time.time()
    basic_results, enhanced_results = run_multiple_simulations(n_runs=n_runs, steps=steps)
    end_time = time.time()
    
    print(f"\n⏱️  Total simulation time: {end_time - start_time:.1f} seconds")
    
    # Create plots
    print("\n📊 Creating comparison plots...")
    create_comparison_plots(basic_results, enhanced_results)
    
    # Print statistics
    print_summary_statistics(basic_results, enhanced_results)
    
    print("\n✅ CONSERVATIVE COOPERATION TEST completed!")
    print("\n🎯 Expected results after bug fixes:")
    print("   • Enhanced cooperation ≥ Basic cooperation (no longer worse)")
    print("   • Better survival rates OR at least not worse") 
    print("   • Reduced cooperation overhead") 
    print("   • The cooperation should help, not hurt!")
    print("\n❓ If still not matching paper:")
    print("   • Might need different parameters (grid size, robot count)")
    print("   • Or paper conditions are very specific")
    print("   • But at least cooperation shouldn't hurt performance anymore")

if __name__ == "__main__":
    main()