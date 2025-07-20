import utils
from environment.box import Box
from environment.nest import Nest

class Robot:
    """Base robot class with common functionality"""

    def __init__(self, e, c, grid):
        self.energy = e
        self.color = utils.validate_color(c)
        self.grid = grid
        self.position = None
        self.carried_box = None
        self.target_box = None
        self.target_nest = None
        self.crit_current = utils.MAX_ENERGY - e  # Initialize criticality
        self.forced_deposit = False
    
    @property
    def is_carrying(self):
        return self.carried_box is not None
    
    @property
    def has_target(self):
        return self.target_box is not None

    @property
    def on_target_box(self):
        if not self.has_target:
            return False
        box_pos = self._find_box_position(self.target_box)
        if box_pos == self.position:
            return box_pos
        else:
            return False

    @property
    def on_target_nest(self):
        if self.target_nest is None:
            print("Warning: robot has no target nest!") if utils.DEBUG_MODE else None
            return False
        return self.target_nest.position == self.position

    @property
    def current_cell(self):
        x, y = self.position
        return self.grid.cells[y][x]
    
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
    
    def move(self):
        """Move randomly to a neighboring cell, if possible"""
        # todo: implement proper Monte Carlo movement
        if self.energy < utils.ENERGY_COST:
            # self.energy = 0
            return False
        
        # todo: Move multiple cells if speed > 1.0
        possible_moves = self.grid.get_neighbors(self.position)

        if possible_moves:
            new_position = utils.seeded_rand.choice(possible_moves)
            self._move_to_position(new_position)
            self.energy -= utils.ENERGY_COST
            return True
        return False    # stuck

    def pickup(self):
        """Take/pick up a box from the current position"""
        if self.is_carrying:  # Already carrying something
            return False

        current_cell = self.current_cell
        if current_cell.box is None:  # Current cell has no box
            return False

        if current_cell.box.status in ["CARRIED", "DEPOSITED"]:     # Can pick only the (free) INITIALIZED boxes
            return False

        box = current_cell.box
        # Pick up the box
        self.carried_box = box
        box.holder = self
        box.set_status("CARRIED")
        box.set_position(self.position)
        # current_cell.remove_box()   # dont remove the box right now, remove when robot moves

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
            # self.energy = 0
            return False

        if target is None:
            return False

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

        # Get all valid moves and find the one with the shortest distance to target
        valid_moves = self.grid.get_neighbors(self.position)

        if not valid_moves:
            print('Robot completely blocked!!') if utils.DEBUG_MODE else None
            return False

        # Find move with the shortest distance to target
        best_move = None
        best_distance = float('inf')
        for move in valid_moves:
            distance = utils.calculate_distance(move, target_pos)
            if distance < best_distance:
                best_distance = distance
                best_move = move

        # Execute the best move
        self.energy -= utils.ENERGY_COST
        self._move_to_position(best_move)
        return True

    def _move_to_position(self, new_position):
        """Helper method to move robot to new position"""
        # new_x, new_y = new_position
        if not utils.validate_position(new_position):
            raise ValueError('Invalid position argument')
            # return False
        # Remove the robot from current cell
        old_x, old_y = self.position
        self.grid.cells[old_y][old_x].remove_robot()

        # update position
        self.position = new_position
        # Add the robot to new cell
        new_x, new_y = new_position
        self.grid.cells[new_y][new_x].add_robot(self)
        # move carried box if robot is carrying one
        if self.is_carrying and self.carried_box is not None and self.carried_box.holder == self:
            old_cell = self.grid.cells[old_y][old_x]
            if old_cell.box == self.carried_box:
                old_cell.remove_box()
            self.carried_box.set_position((new_x, new_y))
        return True
        



    def deposit(self):
        """Deposit carried box in the appropriate nest"""
        current_cell = self.current_cell
        if not self.is_carrying or current_cell.nest is None:
            return False
        
        nest = current_cell.nest

        # Verify nest color matches box color
        if nest.color != self.carried_box.color:
            raise ValueError(f"Trying to deposit {self.carried_box.color} box in {nest.color} nest!")
        

        # SAFETY CHECK: Update target nest if it doesn't match carried box
        if self.target_nest is None or self.target_nest.color != self.carried_box.color:
            for grid_nest in self.grid.nests:
                if grid_nest.color == self.carried_box.color:
                    self.target_nest = grid_nest
                    print(f"Warning: Updated target nest for {self} carrying {self.carried_box.color} box") if utils.DEBUG_MODE else None
                    break
        
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

    def find_nearest_box(self):
        """Find and target the nearest available box"""
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


    def _find_box_position_bk(self, box):
        """Search for the box position"""

        # First, try the cached position
        if hasattr(box, "position") and box.position is not None:
            if utils.validate_position(box.position):
                return box.position
        # If the cached position is unsuccessful, exhaustively search for the box on the grid
        else:
            # utils.DEBUG_MODE and print('exhaustively searching for box position')
            for y in range(self.grid.height):
                for x in range(self.grid.width):
                    cell = self.grid.cells[y][x]
                    if cell.box == box:
                        return x, y
        raise ValueError('error while searching for box')
        # return None


    def _find_box_position(self, box):
        """Search for the box position"""
        if box is None:
            return None
        
        # First, try the cached position
        if hasattr(box, "position") and box.position is not None:
            if utils.validate_position(box.position):
                x, y = box.position
                cell = self.grid.cells[y][x]
                # Verify the box is actually there
                if cell.box == box:
                    return box.position
        
        # If cached position fails, search the grid
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                cell = self.grid.cells[y][x]
                if cell.box == box:
                    # Update the cached position
                    box.set_position((x, y))
                    return (x, y)
        
        # Box not found
        return None
    
    # Common step template
    def step(self):
        """Template method for robot decision-making"""

        # 1) Dead robots do nothing
        if self.energy <= 0:
            return

        # 2) Update criticality at start of each step
        self._evaluate_current_criticality()

        # 3) Avoid camping on nests when empty
        if self.current_cell.nest is not None and not self.is_carrying:
            self.move()
            return

        # 4) Drop any targets that no longer exist
        self._cleanup_invalid_targets()

        # 5) If no boxes remain, wander
        if not self.grid.boxes:
            self.move()
            return

        # 6) Evaluate situation (subclass-specific)
        self._evaluate_situation()

        # 7) Make decision (subclass-specific)
        self._make_decision()

        # 8) Execute the chosen action
        self._execute_action()

    # Target cleanup
    def _cleanup_invalid_targets(self):
        """Remove target if box no longer exists"""
        if self.has_target and not self._box_still_exists(self.target_box):
            self.target_box = None

    def _box_still_exists(self, box):
        """Check if a box still exists on the grid"""
        return box in self.grid.boxes and self._find_box_position(box) is not None

    # situation evaluation
    def _evaluate_situation(self):
        """Evaluate current situation - to be overridden by subclasses"""
        raise NotImplementedError("Make strategic decision - to be overridden by subclasses")

    def _make_decision(self):
        """Make strategic decision - to be overridden by subclasses"""
        raise NotImplementedError("Make strategic decision - to be overridden by subclasses")

    def _execute_action(self):
        # 1) If I need to deposit my current load (either forced or just carrying)...
        if self.forced_deposit or self.is_carrying:
            # Ensure we have a valid target nest
            if self.target_nest is None and self.is_carrying:
                for nest in self.grid.nests:
                    if nest.color == self.carried_box.color:
                        self.target_nest = nest
                        break
                
            if self.target_nest and self.on_target_nest:
                self.deposit()
                self.forced_deposit = False
            elif self.target_nest:
                self.go(self.target_nest)
            else:
                # no valid nest found, just move randomly
                self.move()
        # 2) If I'm not carrying anything, but I have a target box...
        elif self.has_target:
            # Check if target box still exists before going to it
            if not self._box_still_exists(self.target_box):
                self.target_box = None
                return

            if self.on_target_box:
                self.pickup()
            else:
                self.go(self.target_box)
        # 3) Otherwise, fallback to greedy exploration
        else:
            best_box = self._choose_target_box()
            if best_box:
                self.go(best_box)
            else:
                self.move()


    def _choose_target_box(self):
        """Base target selection - to be overridden by subclasses"""
        return self.find_nearest_box()

    def _evaluate_current_criticality(self):
        """C_ri(t) = MaxNe - Ne_ri(t) - Update current criticality"""
        self.crit_current = utils.MAX_ENERGY - self.energy
        
        # If we have a target/carried box, use anticipated criticality instead
        try:
            if self.is_carrying:
                self.crit_current = self.anticipated_criticality(self.carried_box)
            elif self.has_target and self._box_still_exists(self.target_box):
                self.crit_current = self.anticipated_criticality(self.target_box)
        except ValueError:
            # If calculation fails, use basic criticality
            self.crit_current = utils.MAX_ENERGY - self.energy

    def _calculate_t_d(self, box):
        """
        Travel time: time taken to go from current position to box and then to nest
        t_d = (distance(ri,bk) + distance(bk,Nest)) / Speed_ri(t)
        """
        # Special case: if robot is already carrying this box
        if self.is_carrying and self.carried_box == box:
            # Already carrying - just distance to nest
            target_nest = self._find_nest_for_box(box)
            total_distance = utils.calculate_distance(self.position, target_nest.position)
        else:
            # Need to go to box first, then to nest
            box_position = self._find_box_position(box)
            if box_position is None:
                raise ValueError(f"Box {box} doesn't exist!")
            
            target_nest = self._find_nest_for_box(box)
            distance_to_box = utils.calculate_distance(self.position, box_position)
            distance_to_nest = utils.calculate_distance(box_position, target_nest.position)
            total_distance = distance_to_box + distance_to_nest
        
        if self.current_speed <= 0:
            return float('inf')
        
        return total_distance / self.current_speed

    def _calculate_Ne_a(self, box):
        """Calculate Ne_a(bk,t) - anticipated energy after depositing box"""
        t_d = self._calculate_t_d(box)
        if t_d == float('inf'):
            return 0
        
        # Energy after travel: Ne_ri(t + t_d)
        energy_consumed_travel = utils.ENERGY_COST * t_d
        energy_after_travel = max(0, self.energy - energy_consumed_travel)
        
        # Add reward: re_c_ri(bk)
        if self.color == box.color:
            reward = utils.SAME_COLOR_REWARD  # 200
        else:
            reward = utils.DIFFERENT_COLOR_REWARD  # 100
        
        anticipated_energy = energy_after_travel + reward
        
        # Apply bounds as per specification
        if anticipated_energy >= utils.MAX_ENERGY:
            return utils.MAX_ENERGY
        elif anticipated_energy > 0:
            return anticipated_energy
        else:
            return 0
    
    
    
    def _find_nest_for_box(self, box):
        """Find the nest with the same color as the box"""
        for nest in self.grid.nests:
            if nest.color == box.color:
                return nest
        
        raise ValueError(f"No nest found for box color '{box.color}'")


    def _drop_box(self):
        """
        Drop carried box on the current cell so that any robot
        (including the requester) can pick it up.
        """
        if not self.is_carrying:
            return False

        box = self.carried_box

        # 1) Remove link from this robot
        self.carried_box = None

        # 2) Clear any target for this robot if it was targeting this box
        if self.target_box is box:
            self.target_box = None

        # 3) Put the box back into the grid’s box list (if pickup() removed it)
        if box not in self.grid.boxes:
            self.grid.boxes.append(box)

        # 4) Reset the box’s own pointers/status
        box.set_position(self.position)
        box.set_status("INITIALIZED")
        box.holder = None

        return True


    
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



