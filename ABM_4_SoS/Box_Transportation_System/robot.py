import utils
from environment.box import Box
from environment.nest import Nest

class Robot:
    """Base robot class with common functionality"""

    def __init__(self, e, c, grid, x=0, y=0):
        self.energy = e
        self.color = utils.validate_color(c)
        self.grid = grid
        self.position = (x, y)
        self.carried_box = None
        self.target_box = None
        self.target_nest = None
        # Todo: do we actually need the states? 
        self.states = {
            # Note: Multiple states can be true at a time
            "carried": False,   # carrying a box
            "target": False,    # has targeted a box
            "onPosBox": False,  # on same cell as target box
            "onPosNest": False  # on same cell as target nest
        }
    
    @property
    def is_carrying(self):
        return self.carried_box is not None
    
    @property
    def has_target(self):
        return self.target_box is not None

    @property
    def is_on_target_box(self):
        if not self.has_target:
            return False
        box_pos = self._find_box_position(self.target_box)
        return box_pos == self.position if box_pos else False

    @property
    def is_on_target_nest(self):
        if self.target_nest is None:
            return False
        return self.target_nest.position == self.position

    @property
    def _current_cell(self):
        x, y = self.position
        return self.grid.cells[y][x]
    
    @property
    def criticality(self):
        """
        Criticality of robot agent - represents difficulty/stress level.
        Higher values indicate more critical state (lower energy)
        
        Returns:
            float: Criticality value (0 to MAX_ENERGY)
        """
        return utils.MAX_ENERGY - self.energy

    @property
    def current_speed(self):
        """
        Robot speed decreases as energy decreases
        Maximum when fully charged
        """
        speed = utils.BASE_SPEED * (self.energy / utils.MAX_ENERGY)
        # print(speed) 
        return speed
    
    def anticipated_criticality(self, box):
        """
        Anticipated criticality of robot agent for a given box.
        CA_ri(bk, t) = Max_Ne - Ne_a(bk, t)
        
        Returns:
            float: Anticipated criticality value
        """
        Ne_a = self._calculate_Ne_a(box)
        return utils.MAX_ENERGY - Ne_a


    def basic_step(self):
        """Basic behavior without advanced features"""
        if self.energy <= 0:
            return

        self.update_states()

        if self.has_target and self.target_box not in self.grid.boxes:
            self.target_box = None

        if self.is_carrying:
            if self.is_on_target_nest:
                self.deposit()
            else:
                self.go(self.target_nest)
        elif self.has_target:
            if self.is_on_target_box:
                self.pickup()
            else:
                self.go(self.target_box)
        else:
            nearest_box = self.find_nearest_box()
            if nearest_box:
                self.target_box = nearest_box
                self.go(nearest_box)
            else:
                self.move()
    
    def move(self):
        """Move randomly on the grid according to Monte Carlo distribution"""
        if self.energy < utils.ENERGY_COST:
            self.energy = 0
            return False
        
        self.energy -= utils.ENERGY_COST
        
        # Calculate actual distance moved based on current speed
        distance_to_move = self.current_speed
        
        # Move multiple cells if speed > 1.0
        moves_made = 0
        while moves_made < distance_to_move and moves_made < int(distance_to_move) + 1:
            x, y = self.position
            possible_moves = []
            
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    new_x, new_y = x + dx, y + dy
                    if (0 <= new_x < self.grid.width and 
                        0 <= new_y < self.grid.height and
                        self.grid.cells[new_y][new_x].robot is None):
                        possible_moves.append((new_x, new_y))
            
            if possible_moves:
                new_position = utils.seeded_rand.choice(possible_moves)
                self._move_to_position(new_position)
                moves_made += 1
            else:
                break  # Stuck
        
        return moves_made > 0
    
    def deposit(self):
        """Deposit carried box in the appropriate nest"""
        current_cell = self._current_cell
        if not self.is_carrying or current_cell.nest is None:
            return False
        
        nest = current_cell.nest

        # Verify nest color matches box color
        if nest.color != self.carried_box.color:
            raise ValueError(f"Trying to deposit {self.carried_box.color} box in {nest.color} nest!")
        
        # Deposit the box
        nest.deposited_boxes.append(self.carried_box)
        self.carried_box.set_status("DEPOSITED")

        # Remove box from grid.boxes list
        if self.carried_box in self.grid.boxes:
            self.grid.boxes.remove(self.carried_box)

        # calculate reward and bonus
        reward = utils.REWARD_AMOUNT
        if self.color == self.carried_box.color:
            # Bonus for depositing box of same color as robot
            reward += utils.BONUS_AMOUNT
        self.energy = min(self.energy + reward, utils.MAX_ENERGY)   # Since a robot can't exceed MAX_ENERGY
        
        
        # Reset states
        self.carried_box = None
        self.target_nest = None
        self.update_states()
        return True

    def pickup(self):
        """Take/pick up a box from current position"""
        if self.is_carrying:    # Already carrying something
            return False
        
        current_cell = self._current_cell
        if current_cell.box is None:    # Current cell has no box
            return False
        
        box = current_cell.box
        # Pick up the box
        self.carried_box = box
        box.set_status("CARRIED")
        current_cell.remove_box()

        # update states
        self.target_box = None
        self.update_states()

        # Find nest that matches the color for this box
        for nest in self.grid.nests:
            if nest.color == box.color:
                self.target_nest = nest
                break
        return True
    
    def go(self, target):
        """Go towards a specific target (box or nest)"""   
        if self.energy < utils.ENERGY_COST:    # No energy, cannot move
            # robot dies from attempting to move without energy
            self.energy = 0
            return False

        if target is None:
            return False
        
        target_pos = None
        if isinstance(target, Box):
            # find box position on grid
            target_pos = self._find_box_position(target)
            if target_pos is None:
                print(f"  -> Target box no longer exists, clearing target") and utils.DEBUG
                self.target_box = None
                self.update_states()
                return False
        elif isinstance(target, Nest):
            target_pos = target.position
        else:
            raise ValueError(f"Invalid target type: {type(target)}")
        
        # Move one step towards target
        x, y = self.position
        target_x, target_y = target_pos

        # Calculate direction
        dx = 0 if target_x == x else (1 if target_x > x else -1)
        dy = 0 if target_y == y else (1 if target_y > y else -1)

        new_x, new_y = x + dx, y + dy

        # Check if move is valid
        if (0 <= new_x < self.grid.width and
            0 <= new_y < self.grid.height and
            self.grid.cells[new_y][new_x].robot is None):

            self.energy -= utils.ENERGY_COST
            self._move_to_position((new_x, new_y))
            return True
        else:
            # Try alternative moves if direct path is blocked
            alternatives = [(x + dx, y), (x, y + dy)]
            for alt_x, alt_y in alternatives:
                if (0 <= alt_x < self.grid.width and
                    0 <= alt_y < self.grid.height and
                    self.grid.cells[alt_y][alt_x].robot is None):
                    
                    self.energy -= utils.ENERGY_COST
                    self._move_to_position((alt_x, alt_y))
                    return True
        return False
    
    def find_nearest_box(self):
        """Find and target nearest available box"""
        if self.is_carrying:
            return None
        
        min_distance = float("inf")
        nearest_box = None
        x, y = self.position

        for box in self.grid.boxes:
            if box.status == "INITIALIZED":     # Available box
                box_pos = self._find_box_position(box)
                if box_pos:
                    bx, by = box_pos
                    distance = abs(x - bx) + abs(y - by)    # Manhattan distance
                    # Only consider boxes within perception radius
                    if distance <= utils.PERCEPTION_RADIUS and distance < min_distance:
                        min_distance = distance
                        nearest_box = box
        
        if nearest_box:
            self.target_box = nearest_box
            self.update_states()
        
        return nearest_box
    
    def step(self):
        """Main robot decision-making based on perceptions"""
        """Base step - to be overridden by subclasses"""
        # raise NotImplementedError("Subclasses must implement step() method")
        self.basic_step()

    def choose_target_box(self):
        """Base target selection - to be overridden by subclasses"""
        # raise NotImplementedError("Subclasses must implement choose_target_box() method")
        return self.find_nearest_box()

    def _calculate_Ne_a(self, box):
        """
        Calculate Ne_a(bj, t) - anticipated energy after depositing box
        
        Ne_a(bj, t) = {
            Nr_i(t + td) + rec_ri(bk)  if 0 < result < Max_Ne
            Max_Ne                     if result >= Max_Ne  
            0                          else
        }
        """
        t_d = self._calculate_t_d(box)
        if t_d == float('inf'):
            return 0    # robot can't move
        
        # Calculate energy at time t + td (after travel)
        energy_consumed_travel = utils.ENERGY_COST * t_d
        energy_after_travel = max(0, self.energy - energy_consumed_travel)   # lower capping on energy

        # Add reward for depositing the box
        reward = utils.REWARD_AMOUNT
        # check if robots gets bonus for same color
        if self.color == box.color:
            reward += utils.BONUS_AMOUNT
            
        anticipated_energy = energy_after_travel + reward

        # apply bounds as per specification
        if 0 < anticipated_energy < utils.MAX_ENERGY:
            return anticipated_energy
        elif anticipated_energy >= utils.MAX_ENERGY:
            return utils.MAX_ENERGY
        else:
            return 0
    
    def _calculate_t_d(self, box):
        """
        Travel time: time taken to go from current position to box and then to nest
        td = (distance(ri, bk) + distance(bk, Nest)) / Speed_ri(t)
        """
        # Special case: if robot is already carrying this box
        if self.is_carrying and self.carried_box == box:
            # only need to travel to nest
            target_nest = self._find_nest_for_box(box)
            distance_to_nest = utils.manhattan_distance(self.position, target_nest.position)
            total_distance = distance_to_nest
        else:
            # normal case: need to go to box first, then to nest
            box_position = self._find_box_position(box)
            if box_position is None:
                raise ValueError(f"Box {box} doesn't exist!")
            
            # Find appropriate nest for box
            target_nest = self._find_nest_for_box(box)

            distance_to_box = utils.manhattan_distance(self.position, box_position)
            distance_to_nest = utils.manhattan_distance(box_position, target_nest.position)
            total_distance = distance_to_box + distance_to_nest
        
        # avoid division by zero
        if self.current_speed <= 0:
            return float('inf')

        t_d = total_distance / self.current_speed
        return t_d
    
    def _find_nest_for_box(self, box):
        """Find the nest with the same color as the box"""
        for nest in self.grid.nests:
            if nest.color == box.color:     # find nest with SAME color as box
                return nest
        
        # If no matching nest found, this is a configuration error
        raise ValueError(f"No nest found for box color '{box.color}'")

    def _move_to_position(self, new_position):
        """Helper method to move robot to new position"""
        new_x, new_y = new_position
        if (0 <= new_x < self.grid.width and 0 <= new_y < self.grid.height):
            # Remove from current cell
            old_x, old_y = self.position
            self.grid.cells[old_y][old_x].remove_robot()

            # update position
            self.position = new_position
            
            # Add to new cell
            self.grid.cells[new_y][new_x].add_robot(self)

            # Update states based on new position
            self.update_states()
            return True
        return False
    
    def update_states(self):
        """Update robot states based on current position and surroundings"""
        current_cell = self._current_cell

        # Update carried state
        self.states["carried"] = self.carried_box is not None

        # Update target state  
        self.states["target"] = self.target_box is not None

        # Update position-based states (OnPosBox, PostNest)

        # OnPosBox
        if self.has_target and current_cell.box == self.target_box:
            self.states["onPosBox"] = True
        else:
            self.states["onPosBox"] = False

        # PostNest
        if self.target_nest and current_cell.nest == self.target_nest:
            self.states["onPosNest"] = True
        else:
            self.states["onPosNest"] =  False

    def _find_box_position(self, box):
        """Find the position of a box on the grid"""
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                cell = self.grid.cells[y][x]
                if cell.box == box:
                    return (x, y)
        return None
    
    def _get_visible_boxes(self):
        """Get all the boxes in the perception radius"""
        visible_boxes = []
        for box in self.grid.boxes:
            box_pos = self._find_box_position(box)
            if box_pos:     # null check
                distance = utils.manhattan_distance(self.position, box_pos)
                if distance <= utils.PERCEPTION_RADIUS:
                    visible_boxes.append(box)
        # print(visible_boxes)
        return visible_boxes







