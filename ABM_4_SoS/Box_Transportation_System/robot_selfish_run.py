from environment.grid import Grid
from robot_selfish import SelfishRobot
import utils
import matplotlib.pyplot as plt
import numpy as np

class Simulation:
    def __init__(self, num_robots=90, initial_boxes=0):
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
    
    def _create_robots(self, num_robots):
        """Create selfish robots with equal distribution of colors"""
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
                robot = SelfishRobot(
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
        if self.step_count - self.last_box_generation >= self.box_generation_interval:
            # Generate a new box with random color
            color = utils.seeded_rand.choice(utils.COLORS)
            
            # Find an empty cell for the new box
            attempts = 0
            max_attempts = 100
            
            while attempts < max_attempts:
                x = utils.seeded_rand.randint(0, self.grid.width - 1)
                y = utils.seeded_rand.randint(0, self.grid.height - 1)
                
                cell = self.grid.cells[y][x]
                if cell.box is None and cell.robot is None:  # Cell is free (can have nest)
                    from environment.box import Box
                    box = Box(color)
                    box.set_status("INITIALIZED")
                    cell.add_box(box)
                    self.grid.boxes.append(box)
                    self.stats['boxes_generated'] += 1
                    self.last_box_generation = self.step_count
                    break
                
                attempts += 1
            
            if attempts >= max_attempts:
                print(f"Could not place new box at step {self.step_count}") and utils.DEBUG_MODE
    
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
        # Randomize robot order each step to simulate concurrent execution
        active_robots = [robot for robot in self.grid.robots if robot.energy > 0]
        utils.seeded_rand.shuffle(active_robots)  # Random order each step
        
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
        
        return len(active_robots) > 0
    
    def _update_stats(self):
        """Update simulation statistics"""
        self.stats['boxes_deposited'] = sum(len(nest.deposited_boxes) for nest in self.grid.nests)
        self.stats['robot_energies'] = [robot.energy for robot in self.grid.robots]
        self.stats['total_energy_consumed'] = sum(utils.INITIAL_ENERGY - robot.energy for robot in self.grid.robots)
        
    def _should_continue(self):
        """Check if simulation should continue"""
        if self.step_count >= self.max_steps:
            return False
        
        # Stop if all robots are out of energy
        if all(robot.energy <= 0 for robot in self.grid.robots):
            return False
        
        return True
        # return self.step_count < self.max_steps
    
    def plot_results(self):
        """Generate three plots: mean battery, number of boxes, and boxes deposited"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Plot 1: Mean Battery Level over Time
        axes[0, 0].plot(self.time_series['steps'], self.time_series['mean_battery'], 
                    color='blue', linewidth=2)
        axes[0, 0].set_title('Mean Battery Level of Robots Over Time')
        axes[0, 0].set_xlabel('Time Steps')
        axes[0, 0].set_ylabel('Mean Battery Level')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_ylim(0, utils.MAX_ENERGY)
        
        # Plot 2: Number of Boxes on Grid over Time
        axes[0, 1].plot(self.time_series['steps'], self.time_series['num_boxes'], 
                    color='red', linewidth=2)
        axes[0, 1].set_title('Number of Boxes on Grid Over Time')
        axes[0, 1].set_xlabel('Time Steps')
        axes[0, 1].set_ylabel('Number of Boxes')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Boxes Deposited over Time
        axes[1, 0].plot(self.time_series['steps'], self.time_series['boxes_deposited'], 
                    color='green', linewidth=2)
        axes[1, 0].set_title('Cumulative Boxes Deposited Over Time')
        axes[1, 0].set_xlabel('Time Steps')
        axes[1, 0].set_ylabel('Boxes Deposited')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Alive Robots over Time
        axes[1, 1].plot(self.time_series['steps'], self.time_series['alive_robots'], 
                    color='purple', linewidth=2)
        axes[1, 1].set_title('Number of Alive Robots Over Time')
        axes[1, 1].set_xlabel('Time Steps')
        axes[1, 1].set_ylabel('Alive Robots')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Save the plots
        plt.savefig('selfish_robot_simulation_results.png', dpi=300, bbox_inches='tight')
        print("Plots saved as 'selfish_robot_simulation_results.png'")
    
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

if __name__ == "__main__":
    # Run simulation with parameters matching the research conditions
    sim = Simulation(num_robots=90, initial_boxes=0)  # 90 robots (30 each color), no initial boxes
    results = sim.run()