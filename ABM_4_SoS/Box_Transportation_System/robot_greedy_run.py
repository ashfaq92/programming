from environment.grid import Grid
from robot_greedy import RobotGreedy
import utils
import matplotlib.pyplot as plt
import numpy as np
from environment.box import Box

class Simulation:
    def __init__(self, initial_boxes, num_robots):
        # Use proper grid size from utils or default 50x50
        self.grid = Grid(width=50, height=50)  # Standard grid size
        self.step_count = 0
        self.max_steps = 10000  # <-- changed to 10000
        
        # Grid already initializes nests automatically in _initialize_nests_randomly()
        
        # Create initial boxes (if any)
        if initial_boxes > 0:
            self.grid.initialize_boxes(initial_boxes)
        
        # Create robots with specific color distribution
        self._create_robots(num_robots)
        
        # Box generation parameters
        self.box_generation_interval = 3  # 1 box every 3 time units
        self.last_box_generation = 0
        
        # Statistics tracking for plots
        self.stats = {
            'boxes_deposited': 0,
            'total_energy_consumed': 0,
            'robot_energies': [],
            'boxes_generated': 0
        }
        
        # Time series data for plotting
        self.time_series = {
            'steps': [],
            'mean_battery': [],
            'num_boxes': [],
            'boxes_deposited': [],
            'alive_robots': []  # <-- add this
        }
        
        # Initialize time series storage
        self._initialize_time_series()  # Add this line
    
    def _create_robots(self, num_robots):
        """Create non-cooperative robots with equal distribution of colors"""
        if num_robots != 90:
            print(f"Warning: Expected 90 robots, got {num_robots}")
        
        # Create 30 robots of each color (RED, GREEN, BLUE)
        robots_per_color = num_robots // len(utils.COLORS)
        remaining_robots = num_robots % len(utils.COLORS)
        
        for i, color in enumerate(utils.COLORS):
            # Create base number of robots for this color
            robots_to_create = robots_per_color
            
            # Add extra robot if there are remaining robots
            if i < remaining_robots:
                robots_to_create += 1
            
            for j in range(robots_to_create):
                robot = RobotGreedy(
                    e=utils.INITIAL_ENERGY,  # 300 as specified
                    c=color,
                    grid=self.grid
                )
                
                # Use grid's add_robot method (it finds random position automatically)
                if not self.grid.add_robot(robot):
                    print(f"Could not place robot {i*robots_per_color + j} (color: {color})") and utils.DEBUG_MODE
        
        print(f"Created {len(self.grid.robots)} robots: "
              f"RED: {sum(1 for r in self.grid.robots if r.color == 'RED')}, "
              f"GREEN: {sum(1 for r in self.grid.robots if r.color == 'GREEN')}, "
              f"BLUE: {sum(1 for r in self.grid.robots if r.color == 'BLUE')}")
    
    def _generate_box_if_needed(self):
        """Generate a new box every 3 time units"""
        if self.step_count % 3 == 0:
            # Calculate occupancy rate
            # total_entities = len(self.grid.boxes) + len([r for r in self.grid.robots if r.energy > 0])
            # occupancy_rate = total_entities / (self.grid.width * self.grid.height)
            
            # Stop generation if too crowded
            # if occupancy_rate < utils.GRID_OCCUPANCY_RATE:
            #     self._place_new_box()
            # else:
            #     print(f"Occupancy rate is {occupancy_rate:.2f}, skipping box generation") if utils.DEBUG_MODE else None
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
                # print(f"step: {self.step_count} Box generated at ({x}, {y}) (color: {color})") and utils.DEBUG_MODE
                return  # Success - exit
        print(f"Failed to place box (color: {color})") if utils.DEBUG_MODE else None
        # Failed to place after 100 attempts - grid is full, fail silently
    
    def _record_time_series(self):
        """Record data for time series plots"""
        # Calculate mean battery level of active robots
        active_robots = [r for r in self.grid.robots if r.energy > 0]
        mean_battery = np.mean([r.energy for r in active_robots]) if active_robots else 0
        
        # Record data
        self.time_series['steps'].append(self.step_count)
        self.time_series['mean_battery'].append(mean_battery)
        self.time_series['num_boxes'].append(len(self.grid.boxes))
        self.time_series['boxes_deposited'].append(self.stats['boxes_deposited'])
        self.time_series['alive_robots'].append(len(active_robots))  # <-- add this
    
    def step(self):
        """Execute one simulation step"""
        if self.step_count >= self.max_steps:
            return False

        # Randomize robot order each step to simulate concurrent execution
        active_robots = [robot for robot in self.grid.robots if robot.energy > 0]
        utils.seeded_rand.shuffle(active_robots)  # Random order each step
        
        # Execute robots in random order
        for robot in active_robots:
            robot.step()

        # Update statistics FIRST
        self._update_stats()
        
        # Record time series data SECOND
        self._record_time_series()
        
        # Generate new boxes
        self._generate_box_if_needed()
        self.step_count += 1
        
        # CRITICAL FIX: Remove dead robots from grid cells to free up space
        dead_robots = [robot for robot in self.grid.robots if robot.energy <= 0]
        for dead_robot in dead_robots:
            x, y = dead_robot.position
            cell = self.grid.cells[y][x]
            if cell.robot == dead_robot:
                cell.remove_robot()
                if utils.DEBUG_MODE:
                    nest_info = f" (on {cell.nest.color} nest)" if cell.nest else ""
                    print(f"Removed dead robot from {dead_robot.position}{nest_info}")
            self.grid.robots.remove(dead_robot)

        return len(active_robots) > 0
    

        

    
    def plot_results(self):
        """Generate plots with color-specific robot tracking"""
        fig, axes = plt.subplots(2, 2, figsize=(20, 16))
        
        # Plot 1: Average Energy
        axes[0, 0].plot(self.time_series['steps'], self.time_series['avg_energy'], 
                    color='blue', linewidth=0.8, alpha=0.9)
        axes[0, 0].set_title('Average Robot Energy Over Time', fontsize=14)
        axes[0, 0].set_xlabel('Time Steps', fontsize=12)
        axes[0, 0].set_ylabel('Average Energy', fontsize=12)
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_ylim(0, utils.MAX_ENERGY)
        
        # Plot 2: Boxes on Grid
        axes[0, 1].plot(self.time_series['steps'], self.time_series['boxes_on_grid'], 
                    color='red', linewidth=0.8, alpha=0.9)
        axes[0, 1].set_title('Number of Boxes on Grid Over Time', fontsize=14)
        axes[0, 1].set_xlabel('Time Steps', fontsize=12)
        axes[0, 1].set_ylabel('Number of Boxes', fontsize=12)
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Deposited Boxes
        axes[1, 0].plot(self.time_series['steps'], self.time_series['deposited_boxes'], 
                    color='green', linewidth=0.8, alpha=0.9)
        axes[1, 0].set_title('Cumulative Boxes Deposited Over Time', fontsize=14)
        axes[1, 0].set_xlabel('Time Steps', fontsize=12)
        axes[1, 0].set_ylabel('Boxes Deposited', fontsize=12)
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Alive Robots by Color (THIS IS THE NEW FEATURE!)
        axes[1, 1].plot(self.time_series['steps'], self.time_series['alive_robots'], 
                    'k-', linewidth=2, label='Total Alive')
        axes[1, 1].plot(self.time_series['steps'], self.time_series['alive_robots_red'], 
                    'r-', linewidth=1.5, label='Red Robots')
        axes[1, 1].plot(self.time_series['steps'], self.time_series['alive_robots_green'], 
                    'g-', linewidth=1.5, label='Green Robots')
        axes[1, 1].plot(self.time_series['steps'], self.time_series['alive_robots_blue'], 
                    'b-', linewidth=1.5, label='Blue Robots')
        axes[1, 1].set_title('Alive Robots by Color Over Time', fontsize=14)
        axes[1, 1].set_xlabel('Time Steps', fontsize=12)
        axes[1, 1].set_ylabel('Number of Alive Robots', fontsize=12)
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].set_ylim(0, 35)  # Max 30 per color + some headroom
        
        plt.tight_layout()
        plt.show()
        
        # Save the plot
        plt.savefig('greedy_robot_simulation_with_colors.png', 
                    dpi=300, bbox_inches='tight', 
                    facecolor='white', edgecolor='none')
        print("Color-specific plots saved as 'greedy_robot_simulation_with_colors.png'")
    
    def run(self):
        """Run the simulation"""
        print(f"Starting simulation with {len(self.grid.robots)} robots")
        print(f"Initial conditions: Energy={utils.INITIAL_ENERGY}, Max Energy={utils.MAX_ENERGY}")
        print(f"Energy cost={utils.ENERGY_COST}, Perception radius={utils.PERCEPTION_RADIUS}")
        print(f"Box generation: 1 every {self.box_generation_interval} time units")
        print(f"Robot Speed: {utils.BASE_SPEED}")
        
        while self.step():
            if self.step_count % 100 == 0:
                active_robots = sum(1 for robot in self.grid.robots if robot.energy > 0)
                print(f"Step {self.step_count}: {len(self.grid.boxes)} boxes on grid, "
                      f"{self.stats['boxes_deposited']} deposited, "
                      f"{self.stats['boxes_generated']} generated, "
                      f"{active_robots} active robot(s)") and utils.DEBUG_MODE
        
        print(f"\nSimulation completed after {self.step_count} steps")
        print(f"Total boxes generated: {self.stats['boxes_generated']}")
        print(f"Boxes deposited: {self.stats['boxes_deposited']}")
        print(f"Boxes remaining on grid: {len(self.grid.boxes)}")
        print(f"Total energy consumed: {self.stats['total_energy_consumed']}")
        
        # Final robot status
        active_robots = [r for r in self.grid.robots if r.energy > 0]
        print(f"Active robots remaining: {len(active_robots)}")
        if len(active_robots) > 0:
            avg_energy = sum(r.energy for r in active_robots) / len(active_robots)
            print(f"Average energy of active robots: {avg_energy:.2f}")
        
        # Generate plots
        self.plot_results()
        
        return self.stats

    def _initialize_time_series(self):
        """Initialize time series data storage"""
        self.time_series = {
            'steps': [],
            'total_energy': [],
            'avg_energy': [],
            'boxes_on_grid': [],
            'alive_robots': [],
            'alive_robots_red': [],    # Add this
            'alive_robots_green': [],  # Add this
            'alive_robots_blue': [],   # Add this
            'deposited_boxes': [],
            # Keep backward compatibility keys
            'mean_battery': [],
            'num_boxes': [],
            'boxes_deposited': []

        }

    def _update_stats(self):
        """Update simulation statistics"""
        active_robots = [robot for robot in self.grid.robots if robot.energy > 0]
        
        # Count alive robots by color
        alive_red = sum(1 for robot in active_robots if robot.color == 'RED')
        alive_green = sum(1 for robot in active_robots if robot.color == 'GREEN')
        alive_blue = sum(1 for robot in active_robots if robot.color == 'BLUE')
        
        # Update statistics - USE UPDATE() to preserve existing keys
        self.stats.update({
            'total_energy': sum(robot.energy for robot in active_robots),
            'avg_energy': sum(robot.energy for robot in active_robots) / len(active_robots) if active_robots else 0,
            'boxes_on_grid': len(self.grid.boxes),
            'alive_robots': len(active_robots),
            'alive_robots_red': alive_red,
            'alive_robots_green': alive_green, 
            'alive_robots_blue': alive_blue,
            'deposited_boxes': sum(len(nest.deposited_boxes) for nest in self.grid.nests),
            'boxes_deposited': sum(len(nest.deposited_boxes) for nest in self.grid.nests),  # Keep for compatibility
            'robot_energies': [robot.energy for robot in self.grid.robots],
            'total_energy_consumed': sum(utils.INITIAL_ENERGY - robot.energy for robot in self.grid.robots)
            # 'boxes_generated' is preserved from __init__ and _place_new_box()
        })

    def _record_time_series(self):
        """Record current stats in time series"""
        self.time_series['steps'].append(self.step_count)
        self.time_series['total_energy'].append(self.stats['total_energy'])
        self.time_series['avg_energy'].append(self.stats['avg_energy'])
        self.time_series['boxes_on_grid'].append(self.stats['boxes_on_grid'])
        self.time_series['alive_robots'].append(self.stats['alive_robots'])
        self.time_series['alive_robots_red'].append(self.stats['alive_robots_red'])
        self.time_series['alive_robots_green'].append(self.stats['alive_robots_green'])
        self.time_series['alive_robots_blue'].append(self.stats['alive_robots_blue'])
        self.time_series['deposited_boxes'].append(self.stats['deposited_boxes'])
        
        # Also keep the old keys for backward compatibility
        self.time_series['mean_battery'].append(self.stats['avg_energy'])
        self.time_series['num_boxes'].append(self.stats['boxes_on_grid'])
        self.time_series['boxes_deposited'].append(self.stats['deposited_boxes'])

if __name__ == "__main__":
    # Run the simulation with parameters matching the research conditions
    sim = Simulation(initial_boxes=10, num_robots=90)
    results = sim.run()