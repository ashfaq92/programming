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
        self.crit_current = None
    
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
            print("Warning: robot has no target nest!") if utils.DEBUG_MODE else None
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
        Anticipated criticality of a robot agent for a given box.
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
        
        # todo: Move multiple cells if speed > 1.0
        x, y = self.position
        possible_moves = self.grid.get_neighbors(x, y, valid_only=True)

        if possible_moves:
            new_position = utils.seeded_rand.choice(possible_moves)
            self._move_to_position(new_position)
            return True
        return False    # stuck

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
        self.carried_box.clear_position()

        # Remove box from grid.boxes list
        if self.carried_box in self.grid.boxes:
            self.grid.boxes.remove(self.carried_box)

        # calculate reward and bonus
        if self.color == self.carried_box.color:
            # Bonus for depositing box of same color as robot
            reward = utils.SAME_COLOR_REWARD    # 2/3 of max energy = 200
        else:
            reward = utils.DIFFERENT_COLOR_REWARD   # 1/3 of max energy = 100

        self.energy = min(self.energy + reward, utils.MAX_ENERGY)   # Since a robot can't exceed MAX_ENERGY
        
        # Reset states
        self.carried_box = None
        self.target_nest = None
        return True

    def pickup(self):
        """Take/pick up a box from the current position"""
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

        # Clear target (we now have the box)
        self.target_box = None

        # Find the nest that matches the color for this box
        for nest in self.grid.nests:
            if nest.color == box.color:
                self.target_nest = nest
                break
        return True
    
    def go(self, target):
        """Go towards a specific target (box or nest)"""   
        if self.energy < utils.ENERGY_COST:
            self.energy = 0
            return False

        if target is None:
            return False
        
        target_pos = None
        if isinstance(target, Box):
            target_pos = self._find_box_position(target)
            if target_pos is None:
                print(f"Target box no longer exists, clearing target") if utils.DEBUG_MODE else None
                self.target_box = None
                return False
        elif isinstance(target, Nest):
            target_pos = target.position
        else:
            raise ValueError(f"Invalid target type: {type(target)}")
    
        # Get all valid moves and find the one with shortest distance to target
        valid_moves = self.grid.get_neighbors(self.position[0], self.position[1], valid_only=True)

        if not valid_moves:
            print('Robot completely blocked!!') if utils.DEBUG_MODE else None
            return False

        # Find move with shortest distance to target
        best_move = valid_moves[0]  # Start with first valid move
        best_distance = utils.calculate_distance(best_move, target_pos)

        for move in valid_moves:
            distance = utils.calculate_distance(move, target_pos)
            if distance < best_distance:
                best_distance = distance
                best_move = move

        # Execute the best move
        self.energy -= utils.ENERGY_COST
        self._move_to_position(best_move)
        return True

    def find_nearest_box(self):
        """Find and target nearest available box"""
        if self.is_carrying:
            return None
        
        min_distance = float("inf")
        nearest_box = None

        for box in self.grid.boxes:
            if box.status == "INITIALIZED":     # Available box
                box_pos = self._find_box_position(box)
                if box_pos:
                    distance = utils.calculate_distance(self.position, box_pos)
                    # Only consider boxes within perception radius
                    if distance <= utils.PERCEPTION_RADIUS and distance < min_distance:
                        min_distance = distance
                        nearest_box = box
        
        if nearest_box:
            self.target_box = nearest_box
        
        return nearest_box
    
    # MOVED FROM CHILD CLASS - Common step template
    def step(self):
        """Template method for robot decision-making"""
        if self.energy <= 0:
            return

        # Common behavior: clear nest if not carrying
        if self._should_clear_nest():
            self._clear_nest()
            return

        # Common behavior: cleanup invalid targets
        self._cleanup_invalid_targets()
        
        # Main decision loop: if no boxes available, just move randomly
        if not self.grid.boxes:
            self.move()
            return

        # Template method pattern - subclasses override these
        self._evaluate_situation()
        self._make_decision()
        self._execute_action()

    # MOVED FROM CHILD CLASS - Common nest clearing behavior
    def _should_clear_nest(self):
        """Check if robot should move away from nest"""
        current_cell = self._current_cell
        return current_cell.nest is not None and not self.is_carrying

    def _clear_nest(self):
        """Move away from nest to free up the cell"""
        current_cell = self._current_cell
        if utils.DEBUG_MODE:
            print(f"NEST-CLEARING: Robot at {self.position} moving away from {current_cell.nest.color} nest")
        self.move()

    # MOVED FROM CHILD CLASS - Common target cleanup
    def _cleanup_invalid_targets(self):
        """Remove target if box no longer exists"""
        if self._target_box_missing():
            self.target_box = None

    # MOVED FROM CHILD CLASS - Common situation evaluation
    def _evaluate_situation(self):
        """Evaluate current situation - to be overridden by subclasses"""
        self._evaluate_current_criticality()

    def _make_decision(self):
        """Make strategic decision - to be overridden by subclasses"""
        pass  # Base implementation does nothing

    def _execute_action(self):
        """Execute the decided action - common template"""
        if self.is_carrying:
            if self.is_on_target_nest:
                self.deposit()
            else:
                self.go(self.target_nest)
        elif self.has_target:
            # Check if target box still exists before going to it
            if not self._box_still_exists(self.target_box):
                self.target_box = None
                return
                
            if self.is_on_target_box:
                self.pickup()
            else:
                self.go(self.target_box)
        else:
            best_box = self.choose_target_box()
            if best_box:
                self.go(best_box)
            else:
                self.move()

    def choose_target_box(self):
        """Base target selection - to be overridden by subclasses"""
        return self.find_nearest_box()

    # MOVED FROM CHILD CLASS - Common criticality evaluation
    def _evaluate_current_criticality(self):
        """Initialize the current criticality based on carried/target state"""
        self.crit_current = float('inf')
        try:
            if self.is_carrying:
                self.crit_current = self.anticipated_criticality(self.carried_box)
            elif self.has_target and self._box_still_exists(self.target_box):
                self.crit_current = self.anticipated_criticality(self.target_box)
            elif self.has_target:
                # Target box no longer exists, clear it
                self.target_box = None
        except ValueError as e:
            # If we can't calculate criticality, clear the target
            print(f"Warning: {e}. Clearing target.") if utils.DEBUG_MODE else None
            if self.has_target:
                self.target_box = None

    def _calculate_Ne_a(self, box):
        """Calculate Ne_a(bj, t) - anticipated energy after depositing box"""
        t_d = self._calculate_t_d(box)
        if t_d == float('inf'):
            return 0
        
        # Calculate energy at time t + td (after travel)
        energy_consumed_travel = utils.ENERGY_COST * t_d
        energy_after_travel = max(0, self.energy - energy_consumed_travel)

        # Add reward for depositing the box
        if self.color == box.color:
            reward = utils.SAME_COLOR_REWARD
        else:
            reward = utils.DIFFERENT_COLOR_REWARD
            
        anticipated_energy = energy_after_travel + reward

        # apply bounds as per specification
        if 0 < anticipated_energy < utils.MAX_ENERGY:
            return anticipated_energy
        elif anticipated_energy >= utils.MAX_ENERGY:
            return utils.MAX_ENERGY
        else:
            return 0
    
    def _calculate_t_d(self, box):
        """Travel time: time taken to go from current position to box and then to nest"""
        # Special case: if robot is already carrying this box
        if self.is_carrying and self.carried_box == box:
            target_nest = self._find_nest_for_box(box)
            distance_to_nest = utils.calculate_distance(self.position, target_nest.position)
            total_distance = distance_to_nest
        else:
            box_position = self._find_box_position(box)
            if box_position is None:
                raise ValueError(f"Box {box} doesn't exist!")
            
            target_nest = self._find_nest_for_box(box)
            distance_to_box = utils.calculate_distance(self.position, box_position)
            distance_to_nest = utils.calculate_distance(box_position, target_nest.position)
            total_distance = distance_to_box + distance_to_nest
        
        if self.current_speed <= 0:
            return float('inf')

        t_d = total_distance / self.current_speed
        return t_d
    
    def _find_nest_for_box(self, box):
        """Find the nest with the same color as the box"""
        for nest in self.grid.nests:
            if nest.color == box.color:
                return nest
        
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
            return True
        return False

    def _find_box_position(self, box):
        """Fast position lookup using cached position"""
        if hasattr(box, "position") and box.position is not None:
            x, y = box.position
            if 0 <= x < self.grid.width and 0 <= y < self.grid.height:
                return box.position
        return self._find_box_position_exhaustive(box)

    def _find_box_position_exhaustive(self, box):
        """Find the position of a box on the grid"""
        utils.DEBUG_MODE and print('exhaustively searching for box position')
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
            if box_pos:
                distance = utils.calculate_distance(self.position, box_pos)
                if distance <= utils.PERCEPTION_RADIUS:
                    visible_boxes.append(box)
        return visible_boxes

    def _box_still_exists(self, box):
        """Check if a box still exists on the grid"""
        return box in self.grid.boxes and self._find_box_position(box) is not None
    
    def _target_box_missing(self):
        return self.has_target and not self._box_still_exists(self.target_box)