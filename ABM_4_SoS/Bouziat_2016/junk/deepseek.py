import numpy as np
import matplotlib.pyplot as plt
from mesa import Model, Agent
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import math

# Constants
GRID_SIZE = 50
MAX_ENERGY = 300
INITIAL_ROBOTS_PER_COLOR = 30
ENERGY_CONSUMPTION_PER_STEP = 1
PERCEPTION_RADIUS = 3
BOX_SPAWN_INTERVAL = 3
REWARD_SAME_COLOR = 100
REWARD_DIFF_COLOR = 50
COLORS = ['red', 'green', 'blue']


class Box(Agent):
    """Box agent with a color and holder (robot carrying it)"""

    def __init__(self, unique_id, model, color, pos):
        super().__init__(unique_id, model)
        self.color = color
        self.holder = None  # Robot carrying the box
        self.pos = pos


class Nest(Agent):
    """Nest agent with a fixed color and position"""

    def __init__(self, unique_id, model, color, pos):
        super().__init__(unique_id, model)
        self.color = color
        self.pos = pos


class Robot(Agent):
    """Robot agent with energy, color, and state"""

    def __init__(self, unique_id, model, color, pos):
        super().__init__(unique_id, model)
        self.color = color
        self.energy = MAX_ENERGY
        self.pos = pos
        self.carried_box = None
        self.targeted_box = None
        self.state = "free"  # free, carried, targeted, onPosBox, onPosNest
        self.rewards = {c: REWARD_SAME_COLOR if c == color else REWARD_DIFF_COLOR
                        for c in COLORS}
        self.speed = 1.0  # Default speed (decreases with low energy)

    def distance_to(self, pos):
        """Calculate Manhattan distance to a position"""
        return abs(self.pos[0] - pos[0]) + abs(self.pos[1] - pos[1])

    def criticality(self):
        """Calculate current criticality (Eq: Max_Energy - Current_Energy)"""
        return MAX_ENERGY - self.energy

    def anticipated_criticality(self, box, current_time):
        """Calculate anticipated criticality for a box"""
        # Get nest position for the box's color
        nest_pos = next(nest.pos for nest in self.model.schedule.agents
                        if isinstance(nest, Nest) and nest.color == box.color)

        # Calculate travel costs
        dist_to_box = self.distance_to(box.pos)
        dist_to_nest = self.distance_to(nest_pos)
        total_distance = dist_to_box + dist_to_nest

        # Adjust speed based on energy (speed = 1 when energy > 50% of max)
        speed_factor = min(1.0, self.energy / (MAX_ENERGY * 0.5))
        travel_time = total_distance / max(0.1, speed_factor)  # Avoid division by zero

        # Energy consumed during travel
        energy_consumed = travel_time * ENERGY_CONSUMPTION_PER_STEP
        net_energy = self.energy - energy_consumed + self.rewards[box.color]

        # Apply boundaries
        if net_energy > MAX_ENERGY:
            net_energy = MAX_ENERGY
        elif net_energy < 0:
            net_energy = 0

        return MAX_ENERGY - net_energy

    def move_towards(self, target_pos):
        """Move one step towards target position"""
        if self.pos == target_pos:
            return

        # Calculate direction
        dx = target_pos[0] - self.pos[0]
        dy = target_pos[1] - self.pos[1]

        # Move in the dominant direction
        if abs(dx) > abs(dy):
            new_x = self.pos[0] + (1 if dx > 0 else -1)
            new_y = self.pos[1]
        else:
            new_x = self.pos[0]
            new_y = self.pos[1] + (1 if dy > 0 else -1)

        # Ensure new position is within grid bounds
        new_x = max(0, min(new_x, GRID_SIZE - 1))
        new_y = max(0, min(new_y, GRID_SIZE - 1))

        # Move agent
        self.model.grid.move_agent(self, (new_x, new_y))
        self.energy -= ENERGY_CONSUMPTION_PER_STEP

    def take_box(self, box):
        """Take a box if possible"""
        if box.holder is None:
            box.holder = self
            self.carried_box = box
            self.targeted_box = None
            self.state = "carried"
            return True
        return False

    def deposit_box(self):
        """Deposit carried box into matching nest"""
        if not self.carried_box:
            return False

        # Find nest of matching color
        for agent in self.model.grid.get_cell_list_contents([self.pos]):
            if isinstance(agent, Nest) and agent.color == self.carried_box.color:
                # Add reward based on color match
                reward = self.rewards[self.carried_box.color]
                self.energy = min(MAX_ENERGY, self.energy + reward)

                # Remove box from model
                self.model.grid.remove_agent(self.carried_box)
                self.model.schedule.remove(self.carried_box)
                self.carried_box = None
                self.state = "free"
                return True
        return False

    def perceive_boxes(self):
        """Get visible boxes within perception radius"""
        neighbors = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=True, radius=PERCEPTION_RADIUS
        )
        visible_boxes = []
        for pos in neighbors:
            for agent in self.model.grid.get_cell_list_contents([pos]):
                if isinstance(agent, Box) and agent not in visible_boxes:
                    visible_boxes.append(agent)
        return visible_boxes

    def step_non_cooperative(self):
        """Decision algorithm for non-cooperative system"""
        if self.energy <= 0:
            return

        # Update state based on position
        self.update_state()

        # Box selection logic
        best_box = None
        best_crit = float('inf')

        if self.carried_box:
            best_crit = self.anticipated_criticality(self.carried_box, self.model.schedule.time)
        elif self.targeted_box:
            best_crit = self.anticipated_criticality(self.targeted_box, self.model.schedule.time)

        for box in self.perceive_boxes():
            box_crit = self.anticipated_criticality(box, self.model.schedule.time)
            if box_crit < best_crit and (box.holder is None or box.holder == self):
                best_box = box
                best_crit = box_crit

        if best_box and best_box != self.targeted_box:
            self.targeted_box = best_box
            self.state = "targeted"

        # Action selection
        if self.state == "carried":
            nest_pos = next(nest.pos for nest in self.model.schedule.agents
                            if isinstance(nest, Nest) and nest.color == self.carried_box.color)
            if self.pos == nest_pos:
                self.deposit_box()
            else:
                self.move_towards(nest_pos)

        elif self.state == "onPosBox" and self.targeted_box:
            self.take_box(self.targeted_box)

        elif self.state == "targeted" and self.targeted_box:
            if self.pos == self.targeted_box.pos:
                self.state = "onPosBox"
            else:
                self.move_towards(self.targeted_box.pos)

        else:  # Free state
            self.random_move()

    def step_cooperative(self):
        """Decision algorithm for cooperative system (with box exchange)"""
        if self.energy <= 0:
            return

        # Update state based on position
        self.update_state()

        # Box selection logic with cooperation
        best_box = None
        best_crit = float('inf')

        if self.carried_box:
            best_crit = self.anticipated_criticality(self.carried_box, self.model.schedule.time)
        elif self.targeted_box:
            best_crit = self.anticipated_criticality(self.targeted_box, self.model.schedule.time)

        for box in self.perceive_boxes():
            box_crit = self.anticipated_criticality(box, self.model.schedule.time)
            if box_crit < best_crit:
                if box.holder is None:
                    best_box = box
                    best_crit = box_crit
                elif box.holder != self:  # Box held by another robot
                    self.send_cooperation_request(box.holder, box)

        if best_box and best_box != self.targeted_box:
            self.targeted_box = best_box
            self.state = "targeted"

        # Process cooperation requests
        self.process_cooperation_requests()

        # Action selection (same as non-cooperative)
        if self.state == "carried":
            nest_pos = next(nest.pos for nest in self.model.schedule.agents
                            if isinstance(nest, Nest) and nest.color == self.carried_box.color)
            if self.pos == nest_pos:
                self.deposit_box()
            else:
                self.move_towards(nest_pos)

        elif self.state == "onPosBox" and self.targeted_box:
            self.take_box(self.targeted_box)

        elif self.state == "targeted" and self.targeted_box:
            if self.pos == self.targeted_box.pos:
                self.state = "onPosBox"
            else:
                self.move_towards(self.targeted_box.pos)

        else:  # Free state
            self.random_move()

    def update_state(self):
        """Update robot state based on current position and possessions"""
        if self.carried_box:
            nest_pos = next(nest.pos for nest in self.model.schedule.agents
                            if isinstance(nest, Nest) and nest.color == self.carried_box.color)
            if self.pos == nest_pos:
                self.state = "onPosNest"
            else:
                self.state = "carried"
        elif self.targeted_box:
            if self.pos == self.targeted_box.pos:
                self.state = "onPosBox"
            else:
                self.state = "targeted"
        else:
            self.state = "free"

    def random_move(self):
        """Move randomly when no target"""
        possible_steps = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False
        )
        new_position = self.random.choice(possible_steps)
        self.model.grid.move_agent(self, new_position)
        self.energy -= ENERGY_CONSUMPTION_PER_STEP

    def send_cooperation_request(self, holder, box):
        """Send cooperation request to a box holder"""
        if not hasattr(holder, 'inbox'):
            holder.inbox = []

        request = {
            'requester': self,
            'box': box,
            'crit': self.criticality(),
            'ant_crit': self.anticipated_criticality(box, self.model.schedule.time)
        }
        holder.inbox.append(request)

    def process_cooperation_requests(self):
        """Process received cooperation requests"""
        if not hasattr(self, 'inbox') or not self.inbox:
            return

        for request in self.inbox:
            requester = request['requester']
            box = request['box']
            req_crit = request['crit']
            req_ant_crit = request['ant_crit']

            # Carrier's criticality and anticipated criticality
            carrier_crit = self.criticality()
            carrier_ant_crit = self.anticipated_criticality(box, self.model.schedule.time)

            # Decision logic
            if carrier_crit < req_crit and req_ant_crit < req_crit:
                # Accept request: drop the box
                self.carried_box = None
                self.state = "free"
                box.holder = None
            elif carrier_ant_crit < carrier_crit:
                pass  # Implicitly refuse (keep box)
            else:
                # Accept request: drop the box
                self.carried_box = None
                self.state = "free"
                box.holder = None

        self.inbox = []


class ComponentSystem:
    """Component System for SoS model (manages robots of one color)"""

    def __init__(self, color, model):
        self.color = color
        self.model = model
        self.critical = False
        self.a = 1.0  # Sigmoid parameter

    def get_dying_robots(self):
        """Count robots with energy < 1/3 of max"""
        return sum(1 for agent in self.model.schedule.agents
                   if isinstance(agent, Robot) and
                   agent.color == self.color and
                   agent.energy < MAX_ENERGY / 3)

    def goal_criticality(self):
        """Calculate goal criticality using sigmoid function"""
        dying_count = self.get_dying_robots()
        delta = -dying_count  # Δ = goal.value - current_value (goal=0)

        # Sigmoid for EQ goal (from paper)
        return 1 + 1 / (math.exp(delta + self.a)) - 1 / (math.exp(delta - self.a))

    def update(self):
        """Update system state based on criticality"""
        goal_crit = self.goal_criticality()
        self.critical = (goal_crit > 0.5)  # Threshold for critical state

        # If critical, adjust robot rewards
        if self.critical:
            for agent in self.model.schedule.agents:
                if isinstance(agent, Robot) and agent.color == self.color:
                    # Set rewards for all colors to be same as own color
                    agent.rewards = {c: REWARD_SAME_COLOR for c in COLORS}


class BoxTransportModel(Model):
    """Model for box transportation case study"""

    def __init__(self, system_type, width=GRID_SIZE, height=GRID_SIZE):
        super().__init__()
        self.system_type = system_type  # 1, 2, or 3
        self.grid = MultiGrid(width, height, torus=False)
        self.agents.shuffle_do("step")
        self.current_id = 0
        self.component_systems = {}

        # Create nests at fixed positions
        nest_positions = [(10, 10), (40, 10), (25, 40)]
        for i, color in enumerate(COLORS):
            nest = Nest(self.next_id(), self, color, nest_positions[i])
            self.grid.place_agent(nest, nest_positions[i])
            self.schedule.add(nest)

            # Initialize component systems for SoS
            if system_type == 3:
                self.component_systems[color] = ComponentSystem(color, self)

        # Create robots
        for color in COLORS:
            for _ in range(INITIAL_ROBOTS_PER_COLOR):
                pos = self.random_empty_pos()
                robot = Robot(self.next_id(), self, color, pos)
                self.grid.place_agent(robot, pos)
                self.schedule.add(robot)

        # Data collection
        self.datacollector = DataCollector(
            model_reporters={
                "Alive Robots": lambda m: sum(1 for a in m.schedule.agents
                                              if isinstance(a, Robot) and a.energy > 0),
                "Avg Energy": lambda m: np.mean([a.energy for a in m.schedule.agents
                                                 if isinstance(a, Robot)]),
                "Boxes": lambda m: sum(1 for a in m.schedule.agents
                                       if isinstance(a, Box))
            }
        )

    def random_empty_pos(self):
        """Find random empty position on grid"""
        while True:
            x = self.random.randrange(self.grid.width)
            y = self.random.randrange(self.grid.height)
            if self.grid.is_cell_empty((x, y)):
                return (x, y)

    def spawn_box(self):
        """Spawn new box at random position"""
        if self.schedule.time % BOX_SPAWN_INTERVAL == 0:
            pos = self.random_empty_pos()
            color = self.random.choice(COLORS)
            box = Box(self.next_id(), self, color, pos)
            self.grid.place_agent(box, pos)
            self.schedule.add(box)

    def step(self):
        """Model step function"""
        # Update component systems for SoS
        if self.system_type == 3:
            for cs in self.component_systems.values():
                cs.update()

        # Spawn new boxes
        self.spawn_box()

        # Execute agent steps
        self.schedule.step()

        # Collect data
        self.datacollector.collect(self)


# Simulation and Plotting
def run_simulation(system_type, steps=1000):
    """Run simulation for a given system type"""
    model = BoxTransportModel(system_type)
    for _ in range(steps):
        model.step()
    return model.datacollector.get_model_vars_dataframe()


# Run all three systems
results = {}
for system in [1, 2, 3]:
    results[system] = run_simulation(system)

# Plot results
plt.figure(figsize=(15, 10))

# Alive Robots
plt.subplot(3, 1, 1)
for system, df in results.items():
    plt.plot(df.index, df['Alive Robots'], label=f'System {system}')
plt.ylabel('Alive Robots')
plt.legend()

# Average Energy
plt.subplot(3, 1, 2)
for system, df in results.items():
    plt.plot(df.index, df['Avg Energy'], label=f'System {system}')
plt.ylabel('Average Energy')
plt.legend()

# Boxes in Environment
plt.subplot(3, 1, 3)
for system, df in results.items():
    plt.plot(df.index, df['Boxes'], label=f'System {system}')
plt.xlabel('Steps')
plt.ylabel('Boxes in Environment')
plt.legend()

plt.tight_layout()
plt.savefig('sos_simulation_results.png')
plt.show()