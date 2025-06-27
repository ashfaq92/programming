import utils
from robot import Robot

class RobotGreedy(Robot):
    """
    Non-cooperative robot that maximizes its own energy by choosing boxes with best anticipated criticality (lowest CA values)
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _evaluate_situation(self):
        self._evaluate_current_criticality()

    def _make_decision(self):
        """Greedy decision making - override parent method"""
        self._find_and_target_better_box()


    def _find_and_target_better_box(self):
        """Greedy-specific: look for better boxes and abandon current target if found"""
        visible_boxes = self._get_visible_boxes()
        
        for box in visible_boxes:
            if box.status != "INITIALIZED": # ignore uninitialized, CARRIED and DEPOSITED boxes
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

    def _choose_target_box(self):
        """
        Choose the box with the lowest anticipated criticality (the best energy outcome)
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