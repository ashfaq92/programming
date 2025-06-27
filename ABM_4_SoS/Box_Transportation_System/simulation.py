from environment.grid import Grid
from robot_cooperative import RobotCooperative
from robot_greedy import RobotGreedy
import utils
import matplotlib.pyplot as plt
from environment.box import Box


class Simulation:
    # def __init__(self, grid_width, gridh_height, max_steps, initial_boxes, num_robots, robot_type):
    def __init__(self, sim_params: dict, robot_type: str):
        # Use proper grid size from utils or default 50x50

        self.grid =  Grid(sim_params["grid_width"], sim_params["grid_height"]) 
        self.max_steps = sim_params["max_steps"]
        self.robot_type = robot_type
        self.initial_boxes = sim_params["initial_boxes"]
        self.num_robots = sim_params["num_robots"]
        self.num_robots = sim_params["num_robots"]
        self.box_generation_interval =  sim_params["box_gen_interval"] # 1 box every t time units

        
        
        self.step_count = 0

        # Create initial boxes (if any)
        if initial_boxes > 0:
            self.grid.initialize_boxes(initial_boxes)

        # Create robots with specific color distribution
        self._create_robots(self.num_robots, self.robot_type)

        # Box generation parameters

        # Statistics tracking for plots
        self.stats = {
            'boxes_deposited': 0,
            'total_energy_consumed': 0,
            'robot_energies': [],
            'boxes_generated': 0
        }

        # Initialize time series storage
        self._initialize_time_series()

    def _create_robots(self, num_robots, robot_type):
        """Create robots of specified type with equal color distribution"""
        if num_robots != 90:
            print(f"Warning: Expected 90 robots, got {num_robots}")

        # Create robots with equal distribution across colors
        robots_per_color = num_robots // len(utils.COLORS)
        remaining_robots = num_robots % len(utils.COLORS)

        for i, color in enumerate(utils.COLORS):
            robots_to_create = robots_per_color
            if i < remaining_robots:
                robots_to_create += 1

            for j in range(robots_to_create):
                if robot_type == "cooperative":
                    robot = RobotCooperative(
                        e=utils.INITIAL_ENERGY,
                        c=color,
                        grid=self.grid
                    )
                elif robot_type == "greedy":
                    robot = RobotGreedy(
                        e=utils.INITIAL_ENERGY,
                        c=color,
                        grid=self.grid
                    )
                else:
                    raise ValueError(f"Unknown robot type: {robot_type}")
                
                if not self.grid.add_robot(robot):
                    print(f"Could not place {robot_type} robot {i * robots_per_color + j} (color: {color})") if utils.DEBUG_MODE else None

        print(f"Created {len(self.grid.robots)} {robot_type} robots")
        print(f"Color distribution - RED: {sum(1 for r in self.grid.robots if r.color == 'RED')}, "
              f"GREEN: {sum(1 for r in self.grid.robots if r.color == 'GREEN')}, "
              f"BLUE: {sum(1 for r in self.grid.robots if r.color == 'BLUE')}")

    def _generate_box_if_needed(self):
        """Generate a new box every 3 time units"""
        if self.step_count % self.box_generation_interval == 0:
            self._place_new_box()

    def _place_new_box(self):
        """Attempt to place a new box, fail silently if grid is full"""
        color = utils.seeded_rand.choice(utils.COLORS)

        # Try up to 100 random positions
        for _ in range(100):
            x = utils.seeded_rand.randint(0, self.grid.width - 1)
            y = utils.seeded_rand.randint(0, self.grid.height - 1)

            cell = self.grid.cells[y][x]
            if cell.is_empty():
                box = Box(color)
                box.set_status("INITIALIZED")
                cell.add_box(box)
                self.grid.boxes.append(box)
                self.stats['boxes_generated'] += 1
                return  # Success - exit
        
        if utils.DEBUG_MODE:
            print("Box generation failed — grid full.")

    def step(self):
        """Execute one simulation step"""
        if self.step_count >= self.max_steps:
            return False

        # Randomize robot order each step to simulate concurrent execution
        active_robots = [robot for robot in self.grid.robots if robot.energy > 0]
        utils.seeded_rand.shuffle(active_robots)

        # Execute robots in random order
        for robot in active_robots:
            robot.step()

        # Update statistics
        self._update_stats()

        # Record time series data
        self._record_time_series()

        # Generate new boxes
        self._generate_box_if_needed()
        self.step_count += 1

        # Remove dead robots from grid cells to free up space
        dead_robots = [robot for robot in self.grid.robots if robot.energy <= 0]
        for dead_robot in dead_robots:
            x, y = dead_robot.position
            cell = self.grid.cells[y][x]
            if cell.robot == dead_robot:
                cell.remove_robot()
            self.grid.robots.remove(dead_robot)

        return len(active_robots) > 0

    def run(self):
        """Run the simulation"""
        print(f"Starting {self.robot_type} simulation with {len(self.grid.robots)} robots")
        print(f"Initial conditions: Energy={utils.INITIAL_ENERGY}, Max Energy={utils.MAX_ENERGY}")
        print(f"Energy cost={utils.ENERGY_COST}, Perception radius={utils.PERCEPTION_RADIUS}")
        print(f"Box generation: 1 every {self.box_generation_interval} time units")

        while self.step():
            if self.step_count % 100 == 0:
                active_robots = sum(1 for robot in self.grid.robots if robot.energy > 0)
                print(f"Step {self.step_count}: {len(self.grid.boxes)} boxes on grid, "
                      f"{self.stats['deposited_boxes']} deposited, "
                      f"{self.stats['boxes_generated']} generated, "
                      f"{active_robots} active robot(s)")

        print(f"\n{self.robot_type.capitalize()} simulation completed after {self.step_count} steps")
        print(f"Total boxes generated: {self.stats['boxes_generated']}")
        print(f"Boxes deposited: {self.stats['deposited_boxes']}")
        print(f"Boxes remaining on grid: {len(self.grid.boxes)}")
        print(f"Total energy consumed: {self.stats['total_energy_consumed']}")

        # Final robot status
        active_robots = [r for r in self.grid.robots if r.energy > 0]
        print(f"Active robots remaining: {len(active_robots)}")
        
        if len(active_robots) > 0:
            avg_energy = sum(r.energy for r in active_robots) / len(active_robots)
            print(f"Average energy of active robots: {avg_energy:.2f}")

        return self.stats

    def _initialize_time_series(self):
        """Initialize time series data storage"""
        self.time_series = {
            'steps': [],
            'avg_energy': [],
            'boxes_on_grid': [],
            'alive_robots': [],
            'deposited_boxes': []
        }

    def _update_stats(self):
        """Update simulation statistics"""
        active_robots = [robot for robot in self.grid.robots if robot.energy > 0]

        # Calculate averages
        avg_energy = sum(robot.energy for robot in active_robots) / len(active_robots) if active_robots else 0

        # Update statistics
        self.stats.update({
            'avg_energy': avg_energy,
            'boxes_on_grid': len(self.grid.boxes),
            'alive_robots': len(active_robots),
            'deposited_boxes': sum(len(nest.deposited_boxes) for nest in self.grid.nests),
            'total_energy_consumed': sum(utils.INITIAL_ENERGY - robot.energy for robot in self.grid.robots)
        })

    def _record_time_series(self):
        """Record current stats in time series"""
        self.time_series['steps'].append(self.step_count)
        self.time_series['avg_energy'].append(self.stats['avg_energy'])
        self.time_series['boxes_on_grid'].append(self.stats['boxes_on_grid'])
        self.time_series['alive_robots'].append(self.stats['alive_robots'])
        self.time_series['deposited_boxes'].append(self.stats['deposited_boxes'])



def plot_comparison(coop_sim, greedy_sim):
    """Generate comparative plots showing both cooperative and greedy performance"""
    fig, axes = plt.subplots(2, 2, figsize=(20, 16))
    # Plot 1: Average Energy Comparison
    axes[0, 0].plot(coop_sim.time_series['steps'], coop_sim.time_series['avg_energy'],
                    color='blue', linewidth=1.5, alpha=0.9, label='Cooperative')
    axes[0, 0].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['avg_energy'],
                    color='red', linewidth=1.5, alpha=0.9, label='Greedy')
    axes[0, 0].set_title('Average Robot Energy Over Time', fontsize=14)
    axes[0, 0].set_xlabel('Time Steps', fontsize=12)
    axes[0, 0].set_ylabel('Average Energy', fontsize=12)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    axes[0, 0].set_ylim(0, utils.MAX_ENERGY)

    # Plot 2: Boxes on Grid
    axes[0, 1].plot(coop_sim.time_series['steps'], coop_sim.time_series['boxes_on_grid'],
                    color='blue', linewidth=1.5, alpha=0.9, label='Cooperative')
    axes[0, 1].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['boxes_on_grid'],
                    color='red', linewidth=1.5, alpha=0.9, label='Greedy')
    axes[0, 1].set_title('Number of Boxes on Grid Over Time', fontsize=14)
    axes[0, 1].set_xlabel('Time Steps', fontsize=12)
    axes[0, 1].set_ylabel('Number of Boxes', fontsize=12)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    # Plot 3: Deposited Boxes
    axes[1, 0].plot(coop_sim.time_series['steps'], coop_sim.time_series['deposited_boxes'],
                    color='blue', linewidth=1.5, alpha=0.9, label='Cooperative')
    axes[1, 0].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['deposited_boxes'],
                    color='red', linewidth=1.5, alpha=0.9, label='Greedy')
    axes[1, 0].set_title('Cumulative Boxes Deposited Over Time', fontsize=14)
    axes[1, 0].set_xlabel('Time Steps', fontsize=12)
    axes[1, 0].set_ylabel('Boxes Deposited', fontsize=12)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    # Plot 4: Alive Robots Comparison
    axes[1, 1].plot(coop_sim.time_series['steps'], coop_sim.time_series['alive_robots'],
                    color='blue', linewidth=1.5, alpha=0.9, label='Cooperative')
    axes[1, 1].plot(greedy_sim.time_series['steps'], greedy_sim.time_series['alive_robots'],
                    color='red', linewidth=1.5, alpha=0.9, label='Greedy')
    axes[1, 1].set_title('Alive Robots Over Time', fontsize=14)
    axes[1, 1].set_xlabel('Time Steps', fontsize=12)
    axes[1, 1].set_ylabel('Number of Alive Robots', fontsize=12)
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Save the plot
    plt.savefig('robot_strategy_comparison.png',
                dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print("Comparison plots saved as 'robot_strategy_comparison.png'")


def print_final_comparison(coop_stats, greedy_stats):
    """Print final comparison statistics"""
    print("\n" + "="*50)
    print("FINAL COMPARISON")
    print("="*50)
    
    print(f"Boxes Generated:")
    print(f"  Cooperative: {coop_stats['boxes_generated']}")
    print(f"  Greedy:      {greedy_stats['boxes_generated']}")
    
    print(f"\nBoxes Deposited:")
    print(f"  Cooperative: {coop_stats['deposited_boxes']}")
    print(f"  Greedy:      {greedy_stats['deposited_boxes']}")
    
    print(f"\nTotal Energy Consumed:")
    print(f"  Cooperative: {coop_stats['total_energy_consumed']}")
    print(f"  Greedy:      {greedy_stats['total_energy_consumed']}")
    
    # Calculate efficiency metrics
    if coop_stats['boxes_generated'] > 0:
        coop_efficiency = coop_stats['deposited_boxes'] / coop_stats['boxes_generated']
    else:
        coop_efficiency = 0
        
    if greedy_stats['boxes_generated'] > 0:
        greedy_efficiency = greedy_stats['deposited_boxes'] / greedy_stats['boxes_generated']
    else:
        greedy_efficiency = 0
    
    print(f"\nDeposition Efficiency (deposited/generated):")
    print(f"  Cooperative: {coop_efficiency:.3f}")
    print(f"  Greedy:      {greedy_efficiency:.3f}")
    
    if coop_stats['deposited_boxes'] > 0:
        coop_energy_per_box = coop_stats['total_energy_consumed'] / coop_stats['deposited_boxes']
    else:
        coop_energy_per_box = float('inf')
        
    if greedy_stats['deposited_boxes'] > 0:
        greedy_energy_per_box = greedy_stats['total_energy_consumed'] / greedy_stats['deposited_boxes']
    else:
        greedy_energy_per_box = float('inf')
    
    print(f"\nEnergy Cost per Deposited Box:")
    print(f"  Cooperative: {coop_energy_per_box:.2f}")
    print(f"  Greedy:      {greedy_energy_per_box:.2f}")


if __name__ == "__main__":
    print("Running Cooperative vs Greedy Robot Comparison")
    print("="*50)
    
   
    initial_boxes = 10
    sim_params = {
        "grid_width": utils.GRID_WIDTH,
        "grid_height": utils.GRID_HEIGHT,
        "max_steps": utils.MAX_STEPS,
        "initial_boxes": initial_boxes,
        "num_robots": utils.NUM_ROBOTS,
        "box_gen_interval": utils.BOX_GENERATION_RATE
    }


    # Configuration
    print(f"Configuration:")
    print(sim_params)
    
    
    # Run cooperative simulation
    print("\n1. Running Cooperative Simulation...")
    coop_sim = Simulation(sim_params, robot_type="cooperative")
    coop_stats = coop_sim.run()
    
    # Reset random seed for fair comparison
    utils.seeded_rand.seed(utils.RAND_SEED)
    
    # Run greedy simulation
    print("\n2. Running Greedy Simulation...")
    greedy_sim = Simulation(sim_params, robot_type="greedy")
    greedy_stats = greedy_sim.run()
    
    # Compare results
    print_final_comparison(coop_stats, greedy_stats)
    
    # Generate comparison plots
    plot_comparison(coop_sim, greedy_sim)