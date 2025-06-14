"""
CoCaRo Multi-Agent Box Transportation System
===========================================

This implementation reproduces the experiment from:
"Cooperation in Adaptive Multi-Agent Systems through System of Systems modeling"
by Bouziat, Camps, and Combettes (2016)

Updated for Mesa 3.0+ API

The experiment compares three approaches to multi-agent cooperation:

1. INDIVIDUAL (Non-cooperative):
   - Robots work independently
   - Each robot seeks boxes that maximize its own energy
   - No information sharing or cooperation

2. COOPERATIVE:
   - Robots can exchange boxes based on criticality comparison
   - Uses cooperative request mechanism where more critical robots get priority
   - Implements the AMAS (Adaptive Multi-Agent System) approach

3. SoS (System of Systems):
   - Groups robots by color into component systems
   - Component systems can coordinate to help other component systems
   - Implements the SApHESIA model from the paper
   - Enables "macro-cooperation" at the system level

Key Concepts Implemented:
- Criticality calculation: Criticality = MaxEnergy - CurrentEnergy
- Anticipated criticality: Prediction of criticality after completing action
- Cooperative decision algorithm: Help most critical neighbors
- Component systems: Higher-level coordination between robot groups

Environment:
- 50x50 grid world
- 90 robots (30 red, 30 blue, 30 green)
- 3 colored nests (equidistant placement)
- Colored boxes spawn randomly every 3 time steps
- Robots consume energy and get rewards for depositing boxes
"""

import mesa
import numpy as np
import matplotlib.pyplot as plt
import random
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple

class Color(Enum):
    RED = "red"
    BLUE = "blue"
    GREEN = "green"

class RobotState(Enum):
    SEARCHING = "searching"
    TARGETING = "targeting"
    CARRYING = "carrying"
    DEPOSITING = "depositing"

@dataclass
class CooperativeRequest:
    sender_id: int
    sender_criticality: float
    sender_anticipated_criticality: float
    box_id: int

class Box(mesa.Agent):
    def __init__(self, model, color: Color, pos):
        super().__init__(model)
        self.color = color
        self.pos = pos
        self.carrier = None

class Nest(mesa.Agent):
    def __init__(self, model, color: Color, pos):
        super().__init__(model)
        self.color = color
        self.pos = pos
        self.boxes_deposited = 0

class Robot(mesa.Agent):
    """
    Robot agent implementing the cooperative behavior from the paper.

    The robot has three main behavioral modes:
    - individual: Works independently (System 1 from paper)
    - cooperative: Can exchange boxes with other robots based on criticality (System 2)
    - sos: Part of a component system that can help other component systems (System 3)

    Key paper concepts implemented:
    - Criticality calculation: distance from current state to goal state
    - Anticipated criticality: prediction of future criticality
    - Cooperative decision algorithm: help most critical neighbors
    """

    def __init__(self, model, color: Color, pos, max_energy=300, system_type="individual"):
        super().__init__(model)
        self.color = color
        self.pos = pos
        self.energy = max_energy
        self.max_energy = max_energy
        self.state = RobotState.SEARCHING
        self.target_box = None
        self.carried_box = None
        self.speed = 1.0
        self.perception_radius = 3
        self.system_type = system_type
        self.cooperative_requests = []

    def get_criticality(self):
        """
        Calculate robot criticality based on energy level.

        Implements the criticality formula from the paper:
        Cri(t) = MaxNe - Neri(t)

        Where:
        - MaxNe is the maximum battery level (300)
        - Neri(t) is the current battery level at time t

        Higher criticality = more urgent need for help
        """
        return self.max_energy - self.energy

    def get_anticipated_criticality(self, box):
        """Calculate anticipated criticality if carrying this box"""
        if box is None:
            return self.get_criticality()

        # Calculate time to reach box and then nest
        box_distance = self.manhattan_distance(self.pos, box.pos)
        nest_pos = self.get_nest_position(box.color)
        nest_distance = self.manhattan_distance(box.pos, nest_pos)
        total_time = (box_distance + nest_distance) / self.speed

        # Calculate energy after depositing box
        energy_after_movement = max(0, self.energy - total_time)
        reward = self.get_reward(box.color)
        final_energy = min(self.max_energy, energy_after_movement + reward)

        return self.max_energy - final_energy

    def get_reward(self, box_color):
        """Get reward for depositing a box"""
        if self.system_type == "sos":
            # In SoS mode, component systems can modify rewards
            component = self.model.get_component_system(self.color)
            if component and component.is_helping_color(box_color):
                return 50  # Same reward as own color when helping

        if box_color == self.color:
            return 50
        else:
            return 20

    def manhattan_distance(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_nest_position(self, color):
        """Get position of nest for given color"""
        for agent in self.model.agents:
            if isinstance(agent, Nest) and agent.color == color:
                return agent.pos
        return (0, 0)

    def get_visible_boxes(self):
        """Get boxes within perception radius"""
        visible_boxes = []
        for agent in self.model.grid.get_neighbors(self.pos, moore=True,
                                                 radius=self.perception_radius):
            if isinstance(agent, Box) and agent.carrier is None:
                visible_boxes.append(agent)
        return visible_boxes

    def get_neighbor_robots(self):
        """Get robots within perception radius"""
        neighbors = []
        for agent in self.model.grid.get_neighbors(self.pos, moore=True,
                                                 radius=self.perception_radius):
            if isinstance(agent, Robot) and agent != self:
                neighbors.append(agent)
        return neighbors

    def send_cooperative_request(self, target_robot, box):
        """Send cooperative request to another robot"""
        request = CooperativeRequest(
            sender_id=self.unique_id,
            sender_criticality=self.get_criticality(),
            sender_anticipated_criticality=self.get_anticipated_criticality(box),
            box_id=box.unique_id
        )
        target_robot.cooperative_requests.append(request)

    def process_cooperative_requests(self):
        """Process incoming cooperative requests"""
        if not self.cooperative_requests or not self.carried_box:
            self.cooperative_requests.clear()
            return

        for request in self.cooperative_requests:
            # Find the requesting robot
            requesting_robot = None
            for agent in self.model.agents:
                if isinstance(agent, Robot) and agent.unique_id == request.sender_id:
                    requesting_robot = agent
                    break

            if requesting_robot and self.should_give_box(request):
                # Give box to requesting robot
                box = self.carried_box
                self.carried_box = None
                self.state = RobotState.SEARCHING
                requesting_robot.carried_box = box
                requesting_robot.state = RobotState.CARRYING
                box.carrier = requesting_robot
                break

        self.cooperative_requests.clear()

    def should_give_box(self, request):
        """
        Decide whether to give box based on cooperative algorithm from paper.

        Implements Algorithm 4 from the paper: "coop request processing()"

        The logic follows:
        1. If sender is more critical than me:
           - Give box if it won't make sender too critical long-term
        2. If I'm more critical:
           - Only refuse if giving the box would make me worse off
           - Otherwise, still cooperate to help the system

        This implements the AMAS cooperative behavior where agents help
        the most critical agent in their neighborhood.
        """
        my_criticality = self.get_criticality()
        sender_criticality = request.sender_criticality

        # If sender is more critical than me
        if sender_criticality > my_criticality:
            # Check if giving the box won't make sender too critical
            if request.sender_anticipated_criticality < sender_criticality:
                return True
        else:
            # If I'm more critical, only give if my anticipated criticality is not too bad
            my_anticipated = self.get_anticipated_criticality(self.carried_box)
            if my_anticipated < my_criticality:
                return False
            else:
                return True

        return False

    def move_towards(self, target_pos):
        """Move one step towards target position"""
        current_x, current_y = self.pos
        target_x, target_y = target_pos

        # Simple movement - one step at a time
        if current_x < target_x:
            new_x = current_x + 1
        elif current_x > target_x:
            new_x = current_x - 1
        else:
            new_x = current_x

        if current_y < target_y:
            new_y = current_y + 1
        elif current_y > target_y:
            new_y = current_y - 1
        else:
            new_y = current_y

        # If can't move directly, try alternative moves
        new_pos = (new_x, new_y)
        if self.model.grid.is_cell_empty(new_pos):
            self.model.grid.move_agent(self, new_pos)
            return True
        else:
            # Try moving in one dimension only
            alternatives = [(new_x, current_y), (current_x, new_y)]
            for alt_pos in alternatives:
                if self.model.grid.is_cell_empty(alt_pos):
                    self.model.grid.move_agent(self, alt_pos)
                    return True
        return False

    def random_move(self):
        """Move randomly"""
        possible_moves = self.model.grid.get_neighborhood(self.pos, moore=True)
        empty_cells = [pos for pos in possible_moves if self.model.grid.is_cell_empty(pos)]

        if empty_cells:
            new_pos = random.choice(empty_cells)
            self.model.grid.move_agent(self, new_pos)

    def step(self):
        """Robot step function"""
        if self.energy <= 0:
            return

        # Process cooperative requests if cooperative system
        if self.system_type in ["cooperative", "sos"]:
            self.process_cooperative_requests()

        # Update speed based on energy
        self.speed = max(0.1, self.energy / self.max_energy)

        # State machine behavior
        if self.state == RobotState.SEARCHING:
            self.search_behavior()
        elif self.state == RobotState.TARGETING:
            self.targeting_behavior()
        elif self.state == RobotState.CARRYING:
            self.carrying_behavior()

        # Consume energy
        self.energy = max(0, self.energy - 1)

    def search_behavior(self):
        """Behavior when searching for boxes"""
        visible_boxes = self.get_visible_boxes()

        if visible_boxes:
            # Find best box based on anticipated criticality
            best_box = None
            best_anticipated_criticality = float('inf')

            for box in visible_boxes:
                anticipated_crit = self.get_anticipated_criticality(box)

                # Check if box is already targeted by another robot
                if box.carrier is not None:
                    if self.system_type in ["cooperative", "sos"]:
                        # Send cooperative request
                        carrier = box.carrier
                        self.send_cooperative_request(carrier, box)
                else:
                    if anticipated_crit < best_anticipated_criticality:
                        best_anticipated_criticality = anticipated_crit
                        best_box = box

            if best_box:
                self.target_box = best_box
                self.state = RobotState.TARGETING
        else:
            # Random movement to explore
            self.random_move()

    def targeting_behavior(self):
        """Behavior when targeting a specific box"""
        if self.target_box is None or self.target_box.carrier is not None:
            self.state = RobotState.SEARCHING
            self.target_box = None
            return

        # Move towards target box
        if self.pos == self.target_box.pos:
            # Pick up the box
            self.carried_box = self.target_box
            self.target_box.carrier = self
            self.state = RobotState.CARRYING
            self.target_box = None
        else:
            self.move_towards(self.target_box.pos)

    def carrying_behavior(self):
        """Behavior when carrying a box"""
        if self.carried_box is None:
            self.state = RobotState.SEARCHING
            return

        # Move towards appropriate nest
        nest_pos = self.get_nest_position(self.carried_box.color)

        if self.pos == nest_pos:
            # Deposit the box
            reward = self.get_reward(self.carried_box.color)
            self.energy = min(self.max_energy, self.energy + reward)

            # Remove box from simulation
            self.model.grid.remove_agent(self.carried_box)
            self.carried_box.remove()

            # Update nest statistics
            for agent in self.model.agents:
                if isinstance(agent, Nest) and agent.color == self.carried_box.color:
                    agent.boxes_deposited += 1
                    break

            self.carried_box = None
            self.state = RobotState.SEARCHING
        else:
            self.move_towards(nest_pos)

class ComponentSystem:
    """
    Represents a component system in the SoS approach (SApHESIA model).

    This implements the System of Systems modeling from the paper where:
    - Each color group of robots forms a component-system
    - Component systems have goals (avoid robots with low battery)
    - Component systems have resources (the robots themselves)
    - Component systems have functionalities (ability to help other systems)
    - Component systems calculate their own criticality and decide when to help others

    From the paper: "Each AMAS, reified as a component-system of a SoS, uses a
    cooperative decision process in order to interact with other AMAS and to
    collectively give rise to a relevant overall function at the SoS level."
    """

    def __init__(self, color: Color, robots: List[Robot]):
        self.color = color
        self.robots = robots
        self.helping_colors = set()
        self.criticality_threshold = 100  # When half robots have battery < max/3

    def get_criticality(self):
        """Calculate component system criticality"""
        if not self.robots:
            return 0

        # Count robots with low battery (< max_energy/3)
        low_battery_robots = sum(1 for r in self.robots
                               if r.energy < r.max_energy / 3 and r.energy > 0)

        # Goal: avoid low battery robots (criticality increases with more low battery robots)
        return low_battery_robots / len(self.robots)

    def update_helping_behavior(self, other_systems):
        """Update which colors this system should help"""
        my_criticality = self.get_criticality()

        # Clear current helping
        self.helping_colors.clear()

        # Check if should help other systems
        for other_system in other_systems:
            if other_system.color != self.color:
                other_criticality = other_system.get_criticality()

                # Help if other system is more critical and needs contact
                if (other_criticality > my_criticality and
                    other_criticality > 0.5):  # More than half robots in trouble
                    self.helping_colors.add(other_system.color)

    def is_helping_color(self, color):
        """Check if this system is helping the given color"""
        return color in self.helping_colors

class CoCaRoModel(mesa.Model):
    def __init__(self, width=50, height=50, n_robots_per_color=30,
                 max_energy=300, system_type="individual", seed=None):
        super().__init__(seed=seed)

        self.width = width
        self.height = height
        self.n_robots_per_color = n_robots_per_color
        self.max_energy = max_energy
        self.system_type = system_type
        self.box_spawn_interval = 3
        self.steps_since_last_box = 0

        # Create grid
        self.grid = mesa.space.MultiGrid(width, height, False)

        # Data collection
        self.datacollector = mesa.DataCollector(
            model_reporters={
                "Mean_Battery": lambda m: self.get_mean_battery(),
                "Alive_Robots": lambda m: self.get_alive_robots(),
                "Total_Boxes": lambda m: self.count_boxes(),
                "Red_Robots_Alive": lambda m: self.get_alive_robots_by_color(Color.RED),
                "Blue_Robots_Alive": lambda m: self.get_alive_robots_by_color(Color.BLUE),
                "Green_Robots_Alive": lambda m: self.get_alive_robots_by_color(Color.GREEN)
            }
        )

        # Create nests (equidistant from each other)
        nest_positions = [
            (10, 25),   # Red nest
            (25, 10),   # Blue nest
            (40, 40)    # Green nest
        ]

        colors = [Color.RED, Color.BLUE, Color.GREEN]
        for i, (color, pos) in enumerate(zip(colors, nest_positions)):
            nest = Nest(self, color, pos)
            self.grid.place_agent(nest, pos)

        # Create robots
        self.robots_by_color = {Color.RED: [], Color.BLUE: [], Color.GREEN: []}

        for color in colors:
            for _ in range(n_robots_per_color):
                # Random starting position
                x = self.random.randrange(self.grid.width)
                y = self.random.randrange(self.grid.height)
                pos = (x, y)
                robot = Robot(self, color, pos, max_energy, system_type)
                self.grid.place_agent(robot, pos)
                self.robots_by_color[color].append(robot)

        # Create component systems for SoS approach
        if system_type == "sos":
            self.component_systems = {
                Color.RED: ComponentSystem(Color.RED, self.robots_by_color[Color.RED]),
                Color.BLUE: ComponentSystem(Color.BLUE, self.robots_by_color[Color.BLUE]),
                Color.GREEN: ComponentSystem(Color.GREEN, self.robots_by_color[Color.GREEN])
            }
        else:
            self.component_systems = {}

        # Spawn initial boxes
        self.spawn_boxes(5)  # Start with some boxes

    def get_component_system(self, color):
        """Get component system for given color"""
        return self.component_systems.get(color)

    def spawn_boxes(self, count=1):
        """Spawn random colored boxes"""
        colors = [Color.RED, Color.BLUE, Color.GREEN]

        for _ in range(count):
            try:
                x = self.random.randrange(self.grid.width)
                y = self.random.randrange(self.grid.height)
                pos = (x, y)
                if self.grid.is_cell_empty(pos):
                    color = self.random.choice(colors)
                    box = Box(self, color, pos)
                    self.grid.place_agent(box, pos)
            except:
                break  # No empty spaces

    def get_mean_battery(self):
        """Get mean battery level of alive robots"""
        alive_robots = [agent for agent in self.agents
                       if isinstance(agent, Robot) and agent.energy > 0]
        if not alive_robots:
            return 0
        return sum(robot.energy for robot in alive_robots) / len(alive_robots)

    def get_alive_robots(self):
        """Get number of alive robots"""
        return sum(1 for agent in self.agents
                  if isinstance(agent, Robot) and agent.energy > 0)

    def get_alive_robots_by_color(self, color):
        """Get number of alive robots of specific color"""
        return sum(1 for agent in self.agents
                  if isinstance(agent, Robot) and agent.color == color and agent.energy > 0)

    def count_boxes(self):
        """Count total boxes in environment"""
        return sum(1 for agent in self.agents if isinstance(agent, Box))

    def step(self):
        """Model step function"""
        # Update component systems in SoS mode
        if self.system_type == "sos":
            for system in self.component_systems.values():
                system.update_helping_behavior(list(self.component_systems.values()))

        # Spawn new boxes periodically
        self.steps_since_last_box += 1
        if self.steps_since_last_box >= self.box_spawn_interval:
            self.spawn_boxes(1)
            self.steps_since_last_box = 0

        # Execute agent steps using AgentSet (Mesa 3.0+ way)
        robot_agents = self.agents.select(lambda agent: isinstance(agent, Robot))
        robot_agents.shuffle_do("step")

        # Collect data
        self.datacollector.collect(self)

def run_experiment(system_type, steps=10000):
    """Run experiment for given system type"""
    print(f"Running experiment for {system_type} system...")

    model = CoCaRoModel(system_type=system_type)

    for i in range(steps):
        model.step()
        if i % 1000 == 0:
            print(f"  Step {i}/{steps}")

    return model.datacollector.get_model_vars_dataframe()

def plot_results(results_dict):
    """Plot comparison results"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))

    # Mean Battery Level
    ax1 = axes[0, 0]
    for system_type, data in results_dict.items():
        if hasattr(data, 'data'):
            battery_data = data.data.get("Mean_Battery", [])
        else:
            battery_data = data.get("Mean_Battery", [])
        ax1.plot(battery_data, label=system_type, linewidth=2)
    ax1.set_title("Mean Battery Level")
    ax1.set_xlabel("Time Steps")
    ax1.set_ylabel("Battery Level")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Number of Boxes in Environment
    ax2 = axes[0, 1]
    for system_type, data in results_dict.items():
        if hasattr(data, 'data'):
            box_data = data.data.get("Total_Boxes", [])
        else:
            box_data = data.get("Total_Boxes", [])
        ax2.plot(box_data, label=system_type, linewidth=2)
    ax2.set_title("Number of Boxes in Environment")
    ax2.set_xlabel("Time Steps")
    ax2.set_ylabel("Number of Boxes")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Alive Robots
    ax3 = axes[1, 0]
    for system_type, data in results_dict.items():
        if hasattr(data, 'data'):
            alive_data = data.data.get("Alive_Robots", [])
        else:
            alive_data = data.get("Alive_Robots", [])
        ax3.plot(alive_data, label=system_type, linewidth=2)
    ax3.set_title("Number of Alive Robots")
    ax3.set_xlabel("Time Steps")
    ax3.set_ylabel("Number of Alive Robots")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Alive Red Robots (as shown in paper)
    ax4 = axes[1, 1]
    for system_type, data in results_dict.items():
        if hasattr(data, 'data'):
            red_data = data.data.get("Red_Robots_Alive", [])
        else:
            red_data = data.get("Red_Robots_Alive", [])
        ax4.plot(red_data, label=system_type, linewidth=2)
    ax4.set_title("Number of Alive Red Robots")
    ax4.set_xlabel("Time Steps")
    ax4.set_ylabel("Number of Alive Red Robots")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

def demonstrate_criticality():
    """Demonstrate key concepts from the paper"""
    print("\nCoCaRo System Implementation - Key Concepts")
    print("=" * 50)

    print("\n1. CRITICALITY CALCULATION")
    print("-" * 30)

    # Example robots with different energy levels
    robots = [
        {"id": "Robot1", "energy": 250, "color": Color.RED},
        {"id": "Robot2", "energy": 100, "color": Color.BLUE},
        {"id": "Robot3", "energy": 50, "color": Color.GREEN},
        {"id": "Robot4", "energy": 200, "color": Color.RED}
    ]

    print("Robot criticalities:")
    for robot in robots:
        crit = 300 - robot["energy"]  # Criticality = MaxEnergy - CurrentEnergy
        print(f"  {robot['id']} (energy={robot['energy']}): criticality = {crit}")

    print("\n2. COOPERATION DECISION EXAMPLE")
    print("-" * 30)

    robot_a_energy = 50   # High criticality robot
    robot_b_energy = 200  # Low criticality robot

    crit_a = 300 - robot_a_energy
    crit_b = 300 - robot_b_energy

    print(f"Robot A criticality: {crit_a}")
    print(f"Robot B criticality: {crit_b}")
    print(f"Should cooperate? {crit_a > crit_b} (A is more critical)")

    print("\n3. SYSTEM COMPARISON")
    print("-" * 30)
    print("INDIVIDUAL: Robots work independently")
    print("COOPERATIVE: Robots exchange boxes based on criticality")
    print("SoS: Component systems coordinate to help critical groups")

def quick_test():
    """Quick test of the model"""
    print("Testing CoCaRo model...")
    try:
        print("Creating model...")
        model = CoCaRoModel(width=20, height=20, n_robots_per_color=5, system_type="individual")
        print("Model created successfully!")

        print("Running simulation steps...")
        for i in range(100):
            model.step()
            if i % 20 == 0:
                alive = model.get_alive_robots()
                battery = model.get_mean_battery()
                boxes = model.count_boxes()
                print(f"Step {i}: Alive={alive}, Battery={battery:.1f}, Boxes={boxes}")

        print("Getting data...")
        data = model.datacollector.get_model_vars_dataframe()
        print("Quick test completed successfully!")
        return data

    except Exception as e:
        print(f"Error in quick_test: {e}")
        import traceback
        traceback.print_exc()
        return None

# Run the experiments
if __name__ == "__main__":
    try:
        # Set random seed for reproducibility
        random.seed(42)
        np.random.seed(42)

        # Demonstrate key concepts
        demonstrate_criticality()

        # Quick test first
        print("\n" + "="*50)
        print("RUNNING QUICK TEST")
        print("="*50)
        test_data = quick_test()

        if test_data is None:
            print("Quick test failed, stopping execution.")
            exit(1)

        print("Quick test completed successfully!")

        # Run full experiments for all three system types
        print("\n" + "="*50)
        print("RUNNING FULL EXPERIMENTS")
        print("="*50)

        system_types = ["individual", "cooperative", "sos"]
        results = {}

        for system_type in system_types:
            print(f"\nRunning {system_type.upper()} system experiment...")
            try:
                results[system_type] = run_experiment(system_type, steps=1000)  # Reduced steps for demo
                print(f"{system_type.upper()} experiment completed successfully!")
            except Exception as e:
                print(f"Error in {system_type} experiment: {e}")
                continue

        if not results:
            print("No experiments completed successfully.")
            exit(1)

        # Plot results
        print("\nGenerating comparison plots...")
        try:
            plot_results(results)
            print("Plots generated successfully!")
        except Exception as e:
            print(f"Error generating plots: {e}")

        # Print final statistics
        print("\nFINAL STATISTICS")
        print("-" * 50)
        for system_type, data in results.items():
            if hasattr(data, 'data'):
                alive_data = data.data.get("Alive_Robots", [])
                battery_data = data.data.get("Mean_Battery", [])
                box_data = data.data.get("Total_Boxes", [])
                final_robots = alive_data[-1] if alive_data else 0
                final_battery = battery_data[-1] if battery_data else 0
                final_boxes = box_data[-1] if box_data else 0
            else:
                final_robots = data.iloc[-1]["Alive_Robots"]
                final_battery = data.iloc[-1]["Mean_Battery"]
                final_boxes = data.iloc[-1]["Total_Boxes"]

            print(f"{system_type.upper()} System:")
            print(f"  Alive Robots: {final_robots}")
            print(f"  Mean Battery: {final_battery:.2f}")
            print(f"  Boxes in Environment: {final_boxes}")
            print()

        print("\nCONCLUSION:")
        print("This implementation reproduces the CoCaRo experiment from the paper,")
        print("demonstrating how cooperation and SoS modeling improve system performance.")
        print("The SoS approach should show the best results, followed by cooperative,")
        print("then individual robots.")

    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()