import utils
from robot import Robot

class RobotNonCooperative(Robot):
    """
    Non-cooperative robot that maximizes its own energy by choosing boxes with best anticipated criticality (lowest CA values)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.forced_deposit = False
        self.crit_current = None

    def _box_still_exists(self, box):
        """Check if a box still exists on the grid"""
        return box in self.grid.boxes and self._find_box_position(box) is not None

    def evaluate_current_criticality(self):
        # Initialize the current criticality based on carried/target state
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

    def find_and_target_better_box(self):
        # update visible boxes (within perception radius)
        visible_boxes = self._get_visible_boxes()
        # check all visible boxes for better
        for box in visible_boxes:
            if box.status != "INITIALIZED":  # skip uninitialized boxes
                continue
            
            # Double-check box still exists before calculating criticality
            if not self._box_still_exists(box):
                continue
                
            try:
                crit_temp = self.anticipated_criticality(box)
            except ValueError as e:
                # Box was removed between checks, skip it
                print(f"Warning: {e}. Skipping box.") if utils.DEBUG_MODE else None
                continue

            if crit_temp < self.crit_current:
                # found a SIGNIFICANTLY better box
                self.crit_current = crit_temp
                # target the new better box
                self.target_box = box
                if self.is_carrying:
                    self.forced_deposit = True

    def act_on_current_state(self):
        # movement logic: execute movement based on the current state
        # If the robot was already carrying a box, it drops it off(deposits) before targeting the new one.
        if self.forced_deposit or self.is_carrying:
            if utils.DEBUG_MODE and self.target_nest:
                nest_x, nest_y = self.target_nest.position
                nest_cell = self.grid.cells[nest_y][nest_x]
                print(f"Robot at {self.position}, target_nest at {self.target_nest.position}, "
                      f"nest_cell.robot: {nest_cell.robot is not None}, "
                      f"nest_cell.nest: {nest_cell.nest is not None}, "
                      f"positions match: {self.target_nest.position == self.position}, "
                      f"on_nest: {self.is_on_target_nest}")

            if self.is_on_target_nest:
                self.deposit()
                if self.forced_deposit:
                    self.forced_deposit = False
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
        # no target - find the best available box
        else:
            best_box = self.choose_target_box()
            if best_box:
                self.go(best_box)
            else:
                self.move()

    def _target_box_missing(self):
        return self.has_target and not self._box_still_exists(self.target_box)

    def step(self):
        "main decision-making for non-cooperative robot"
        if self.energy <= 0:    # as long as the robot has energy
            return

        # CRITICAL FIX: If robot is on a nest cell but not carrying, move away to free the cell!
            # CRITICAL FIX: If robot is on a nest cell but not carrying, move away to free the cell!
        current_cell = self._current_cell
        if current_cell.nest is not None and not self.is_carrying:
            # Robot is on a nest but has no box - free up the cell for others
            if utils.DEBUG_MODE:
                print(f"NEST-CLEARING: Robot at {self.position} moving away from {current_cell.nest.color} nest")
            self.move()
            return

        # Remove target if box no longer exists
        if self._target_box_missing():
            self.target_box = None
        
        # Main decision loop: if no boxes available, just move randomly
        if not self.grid.boxes:
            self.move()
            return

        self.evaluate_current_criticality()
        self.find_and_target_better_box()
        self.act_on_current_state()

    def choose_target_box(self):
        """
        Choose the box with the lowest anticipated criticality (best energy outcome)
        """
        if self.is_carrying:
            return None
        
        visible_boxes = self._get_visible_boxes()
        best_box = None
        crit_best = float('inf')

        for box in visible_boxes:
            if box.status == "INITIALIZED" and self._box_still_exists(box):     # Only consider available boxes
                try:
                    crit = self.anticipated_criticality(box)
                except ValueError:
                    continue
                if crit < crit_best:
                    crit_best = crit
                    best_box = box
        if best_box:
            self.target_box = best_box
        
        return best_box