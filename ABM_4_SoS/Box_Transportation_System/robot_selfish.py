import utils
from robot import Robot

class SelfishRobot(Robot):
    """
    Selfish robot that maximizes its own energy by choosing boxes with best anticipated criticality (lowest CA values)
    """
    
    def step(self):
        "main decision making for selfish robot"
        if self.energy <= 0:
            return
        
        # update states first
        self.update_states()

        # Remove target if box no longer exists
        if self.has_target and self.target_box not in self.grid.boxes:
            self.target_box = None
            self.update_states()
        
        # Main decision loop: while there are boxes available
        if len(self.grid.boxes) == 0:
            return  # no boxes available
        
        # Initialize current criticality based on carried/target state
        if self.is_carrying:
            try:
                crit_current = self.anticipated_criticality(self.carried_box)
            except ValueError:
                crit_current = float('inf')
        elif self.has_target:
            try:
                crit_current = self.anticipated_criticality(self.target_box)
            except ValueError:
                self.target_box = None
                self.update_states()
                crit_current = float('inf')
        else:
            crit_current = float('inf')   # no current box commitment
        
        # update visible boxes (within perception radius)
        visible_boxes = self._get_visible_boxes()

        # If no visible boxes but boxes exist on grid, be more aggressive in searching
        if not visible_boxes and len(self.grid.boxes) > 0:
            # Lower the improvement threshold when desperate for any box
            improvement_threshold = 10  # Much lower threshold when no alternatives
        else:
            improvement_threshold = 50  # Original threshold when boxes are visible
        
        # check all visible boxes for better - BUT require significant improvement
        for box in visible_boxes:
            if box.status != "INITIALIZED":
                continue
            try:
                crit_temp = self.anticipated_criticality(box)
            except ValueError:
                continue

            # CRITICAL FIX: Require 50+ point improvement to justify switching
            if crit_temp < crit_current - improvement_threshold:
                # found a SIGNIFICANTLY better box
                crit_current = crit_temp

                # Only switch if robot has energy buffer to afford the waste
                if self.is_carrying and self.energy > 100:  # Energy buffer check
                    self._deposit_carried_box()
                elif not self.is_carrying:  # Not carrying anything, safe to switch
                    pass  # Continue to target assignment
                else:
                    continue  # Skip switching if carrying and low energy
                
                # target the new better box
                self.target_box = box
                self.update_states()
                break  # Take first significantly better option
        
        # execute movement based on current state
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
            # no target - find the best available box
            best_box = self.choose_target_box()
            if best_box:
                self.go(best_box)
            else:
                # More active exploration when no immediate targets
                if self.energy > 50:  # Only if enough energy
                    self.move()
                # Else conserve energy by staying still
            

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
            if box.status == "INITIALIZED":     # Only consider avaialble boxes
                try:
                    crit = self.anticipated_criticality(box)
                except ValueError:
                    continue
                if crit < crit_best:
                    crit_best = crit
                    best_box = box
        if best_box:
            self.target_box = best_box
            self.update_states()
        
        return best_box
    
    def _deposit_carried_box(self):
        """Deposit the currently carried box on the ground (not in nest)"""
        if not self.is_carrying:
            return
        
        current_cell = self._current_cell
        deposited = False
        
        # Try current cell first
        if current_cell.box is None:
            current_cell.add_box(self.carried_box)
            self.carried_box.set_status("INITIALIZED")
            deposited = True
        else:
            # Try nearby cells
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    new_x = self.position[0] + dx
                    new_y = self.position[1] + dy
                    if (0 <= new_x < self.grid.width and 0 <= new_y < self.grid.height):
                        nearby_cell = self.grid.cells[new_y][new_x]
                        if nearby_cell.box is None:
                            nearby_cell.add_box(self.carried_box)
                            self.carried_box.set_status("INITIALIZED")
                            deposited = True
                            break
                if deposited:
                    break
        
        if deposited:
            # Successfully placed box
            self.carried_box = None
            self.target_nest = None
            self.update_states()
        else:
            # CRITICAL: Can't place box anywhere - keep carrying it!
            # Don't drop the box, find somewhere else to go
            pass  # Robot keeps the box and continues