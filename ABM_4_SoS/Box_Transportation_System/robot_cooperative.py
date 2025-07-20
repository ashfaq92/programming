import utils
from robot import Robot


class RobotCooperative(Robot):
    """
    Cooperative robot: negotiates direct box exchanges with peers
    based on current and anticipated criticalities.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.coop_reqs = []     # store incoming cooperation requests

    def _evaluate_situation(self):
        """Cooperative robots evaluate situation by updating criticality"""
        self._evaluate_current_criticality()  # This updates self.crit_current

    def _make_decision(self):
        """Cooperative decision making"""
        # 1) Process any pending cooperation requests FIRST
        self.process_coop_reqs()
        
        # 2) Look for better boxes or cooperation opportunities
        self._find_and_target_better_box()

    def _find_and_target_better_box(self):
        """EXACT research algorithm - no modifications"""
        visible_boxes = self._get_visible_boxes()
        
        # Current criticality baseline
        CA_current = self.crit_current
        
        for box in visible_boxes:
            if box.status == "DEPOSITED":
                continue
                
            if not self._box_still_exists(box):
                continue
                
            # Calculate temp_CA = criticality_if_i_had(bₖ)
            try:
                temp_CA = self.anticipated_criticality(box)
            except ValueError:
                continue
                
            # if temp_CA < CA_current: (EXACT from paper)
            if temp_CA < CA_current:
                if box.status == "CARRIED":  # if bₖ is held by rⱼ:
                    box_holder = self._find_box_holder(box)
                    if box_holder and isinstance(box_holder, RobotCooperative):
                        # send Co-op request to rⱼ with (Cᵢ_current, temp_CA)
                        self.send_coop_req(box_holder, box)
                else:  # box is free
                    # update CA_current ← temp_CA
                    CA_current = temp_CA
                    self.crit_current = temp_CA
                    # choose bₖ as new target
                    self.target_box = box
                    # if carrying another box: deposit it
                    if self.is_carrying:
                        self.forced_deposit = True
                    break  # Found a better option

    def send_coop_req(self, box_holder, box):
        """
        Send a cooperation request - but only if the holder still has the box
        """

        # double-check if the box holder still has the box
        if box_holder.carried_box != box:
            return
        
        req = {
            "sender": self,
            "c_sender": self.crit_current,
            "ca_sender": self.anticipated_criticality(box),
            "box": box
        }
        box_holder.receive_coop_req(req)
        print(f"DEBUG: {self} requesting {box.color} box from {box_holder}") if utils.DEBUG_MODE else None

    def receive_coop_req(self, req):
        """
        Called by another robot to register a coop request.
        It will be processed later during decision-making.
        """
        self.coop_reqs.append(req)

    def process_coop_reqs(self):
        """Process cooperation requests according to research specification"""
        if not self.coop_reqs:
            return
        
        # Process valid requests
        valid_reqs = [req for req in self.coop_reqs if self.carried_box == req["box"]]
        if not valid_reqs:
            self.coop_reqs.clear()
            return

        req = valid_reqs[0]
        sender = req["sender"]
        C_sender = req["c_sender"]        # Sender's current criticality
        CA_sender = req["ca_sender"]      # Sender's anticipated criticality with box
        box = req["box"]
        
        # Use current criticality (already updated in _evaluate_situation)
        C_mine = self.crit_current  # Use consistent criticality
        
        # Research algorithm decision logic:
        if C_mine < C_sender:  # I'm less critical than requester
            if CA_sender < C_sender:  # Requester would improve by taking box
                self._accept_exchange(sender, box)
            else:  # Requester would get worse - refuse
                self._refuse_exchange(sender, box)
        else:  # I'm more/equally critical than requester
            try:
                CA_mine = self.anticipated_criticality(box)  # My anticipated criticality
                if CA_mine < C_mine:  # Keeping box would improve my situation
                    self._refuse_exchange(sender, box)
                else:  # Keeping box would make me worse - give it away
                    self._accept_exchange(sender, box)
            except ValueError:
                self._refuse_exchange(sender, box)
    
        self.coop_reqs.clear()



    def _find_box_holder(self, box):
        """Find which robot is holding a specific box"""
        if box.holder:
            return box.holder
    
        # Search all robots
        for robot in self.grid.robots:
            if robot.carried_box == box:
                return robot
    
        # Box holder not found - return None instead of raising exception
        print(f"Warning: Cannot find box holder for {box}") if utils.DEBUG_MODE else None
        return None

    def _accept_exchange(self, requester, box):
        """Accept exchange by DIRECTLY transferring box to requester"""
        assert self.carried_box is box, "Can't accept exchange for a box I don't hold"
        print(f"DEBUG: EXCHANGE! {self} giving {box.color} box directly to {requester}") if utils.DEBUG_MODE else None
        
        # Clear my state
        self.carried_box = None
        self.target_nest = None
        self.target_box = None
        
        # DIRECT TRANSFER to requester
        requester.carried_box = box
        box.holder = requester
        box.set_status("CARRIED")
        box.set_position(requester.position)
        
        # Set requester's target nest
        for nest in self.grid.nests:
            if nest.color == box.color:
                requester.target_nest = nest
                break
    
        requester.target_box = None
    
        # Update both robots' criticality
        self._evaluate_current_criticality()
        requester._evaluate_current_criticality()
        
        print(f"{self} → gave {box.color} box directly to {requester}") if utils.DEBUG_MODE else None


    def _refuse_exchange(self, sender, box):
        print(f"{self} refused to exchange {box} with {sender}") if utils.DEBUG_MODE else None
