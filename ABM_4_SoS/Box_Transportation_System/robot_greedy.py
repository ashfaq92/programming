import utils
from robot import Robot

class RobotGreedy(Robot):
    """
    Non-cooperative robot that maximizes its own energy by choosing boxes with best anticipated criticality (lowest CA values)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.forced_deposit = False

    def _make_decision(self):
        """Greedy-specific decision making - override parent method"""
        self.find_and_target_better_box()

    def _execute_action(self):
        """Override parent to handle forced deposit logic"""
        # Greedy-specific logic: forced deposit handling
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
        else:
            best_box = self.choose_target_box()
            if best_box:
                self.go(best_box)
            else:
                self.move()

    def find_and_target_better_box(self):
        """Greedy-specific: look for better boxes and abandon current target if found"""
        visible_boxes = self._get_visible_boxes()
        
        for box in visible_boxes:
            if box.status != "INITIALIZED":
                continue
            
            # Double-check box still exists before calculating criticality
            if not self._box_still_exists(box):
                continue
                
            try:
                crit_temp = self.anticipated_criticality(box)
            except ValueError as e:
                print(f"Warning: {e}. Skipping box.") if utils.DEBUG_MODE else None
                continue

            if crit_temp < self.crit_current:
                # found a SIGNIFICANTLY better box
                self.crit_current = crit_temp
                # target the new better box
                self.target_box = box
                if self.is_carrying:
                    self.forced_deposit = True

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
            if box.status == "INITIALIZED" and self._box_still_exists(box):
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