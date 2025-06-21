def choose_target_box(self):
    best_box = None
    min_CA = float('inf')
    for box in self.grid.boxes:
        ca = self.anticipated_criticality(box)
        if ca < min_CA:
            min_CA = ca
            best_box = box
    return best_box



def step(self):
    """Main robot decision-making based on perceptions"""
    if self.energy <= 0:
        utils.DEBUG_MODE and print(f"  -> Robot at {self.position} is dead!")
        return  # Only return if truly dead

    # Update states first
    self.update_states()

    # Validate target_box still exists AND is within perception
    if self.has_target:
        box_pos = self._find_box_position(self.target_box)
        if box_pos is None:
            utils.DEBUG_MODE and print(f"  -> Target box disappeared, clearing target")
            self.target_box = None
            self.states["target"] = False
        else:
            # check if target is still within perception radius
            x, y = self.position
            bx, by = box_pos
            distance = abs(x - bx) + abs(y - by)
            if distance > utils.PERCEPTION_RADIUS:
                utils.DEBUG_MODE and print(f"  -> Target box out of perception range, clearing target")
                self.target_box = None
                self.states["target"] = False

    # Robot perception logic
    if self.is_carrying:
        # print(f"Robot at {self.position} carrying {self.carried_box.color} box, target nest: {self.target_nest.position if self.target_nest else 'None'}")
        if self.is_on_target_nest:
            utils.DEBUG_MODE and print("  -> Depositing")
            self.deposit()
        else:
            utils.DEBUG_MODE and print("  -> Going to nest")
            # Find appropriate nest for the carried box
            if not self.target_nest or self.target_nest.color != self.carried_box.color:
                self.target_nest = self._find_nest_for_box(self.carried_box)
            if self.target_nest:
                self.go(self.target_nest)
    elif self.has_target:
        if self.is_on_target_box:
            utils.DEBUG_MODE and print("  -> Taking box")
            self.pickup()
        else:
            utils.DEBUG_MODE and print("  -> Going to target box")
            self.go(self.target_box)
    else:
        # Find nearest box when free
        utils.DEBUG_MODE and print("  -> Searching for boxes")
        nearest_box = self.find_nearest_box()
        if nearest_box:
            utils.DEBUG_MODE and print(f"     Found box at distance, targeting it")
            self.go(nearest_box)
        else:
            utils.DEBUG_MODE and print("  -> Moving randomly (no boxes found)")
            self.move()