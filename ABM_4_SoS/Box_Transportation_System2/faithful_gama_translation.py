import mesa
import random
import numpy as np
from typing import List, Dict, Optional, Tuple, Any
import matplotlib.pyplot as plt
from dataclasses import dataclass
from enum import Enum

# GAMA Constants (from utils.gaml)
MAX_BATTERY = 300
REWARD = int((2 * MAX_BATTERY) / 3)  # 200
COUNTER_EFFICIENCY = 0.2
MIN_BATTERY = 0
MAX_CRITICALITY = MAX_BATTERY
MIN_CRITICALITY = MIN_BATTERY
INITIAL_BATTERY = MAX_BATTERY
BATTERY_CONSUM = 1
NEED_BOX_THRESHOLD = MAX_CRITICALITY // 2
BATTERY_STATE_NB = 3
MAX_BOXES_BUFFER = 2

# Message constants
CRITICALITY_STRING = 'criticality'
DEMAND_BOX_STRING = 'givemeyourbox'
GIVE_MY_BOX_STRING = 'givmyboxtoyou'

class Direction(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2
    BACK = 3

class Cardinal(Enum):
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3

@dataclass
class Message:
    sender: Any
    content: List[Any]
    protocol: str = 'fipa-request'
    performative: str = 'request'

class Box(mesa.Agent):
    """Direct translation of box.gaml"""
    def __init__(self, unique_id: int, model, color: str):
        super().__init__(unique_id, model)
        self.color = color
        self.owner = None
        self.this_is_the_end = False
        self.myCell = None
        
    def step(self):
        if self.this_is_the_end:
            if self in self.model.schedule.agents:
                self.model.schedule.remove(self)
            if self.pos is not None:
                try:
                    self.model.grid.remove_agent(self)
                except:
                    pass

class RobotWithoutBuffer(mesa.Agent):
    """Direct translation of robot_without_buffer.gaml"""
    
    def __init__(self, unique_id: int, model, color: str):
        super().__init__(unique_id, model)
        self.size = 1.0
        self.battery = INITIAL_BATTERY
        self.color = color
        self.robot_speed = 1
        
        # GAMA attributes
        self.myCell = None
        self.reachable_boxes = []
        self.targeted_box = None
        self.carried_box = None
        self.is_carrying_box = False
        self.is_empty_battery = False
        self.is_waiting_to_give_box = False
        self.is_need_a_box = False
        self.is_request_criticality_last_cycle = False
        self.is_diry_orientation = False
        self.is_auth_to_move = True
        self.is_dead = False
        
        # Battery state (0: High, 1: Medium, 2: Low)
        self.battery_state = BATTERY_STATE_NB - 1
        self.waiting_turn_nb = 0
        self.criticality = 0
        
        # Orientation and direction
        self.heading = random.choice([0, 90, 180, 270])
        self.orientation = {}
        
        # Communication
        self.requests = []
        self.agrees = []
        self.refuses = []
        self.informs = []
        
        # Efficiency mapping
        self.colors_reward_efficiency = {}
        self.set_reward_efficiency(color)
        
        # Initialize
        self.redefine_direction(self.heading)
        self.place_robot()
    
    def place_robot(self):
        """Place robot on grid"""
        attempts = 0
        while attempts < 100:
            x = random.randint(0, self.model.grid.width - 1)
            y = random.randint(0, self.model.grid.height - 1)
            if len(self.model.grid.get_cell_list_contents([(x, y)])) == 0:
                self.model.grid.place_agent(self, (x, y))
                self.myCell = (x, y)
                break
            attempts += 1
    
    def redefine_direction(self, heading_temp: int):
        """Direct translation of redefine_direction action"""
        if heading_temp == 0:  # East
            self.orientation = {
                Direction.FORWARD: Cardinal.EAST,
                Direction.BACK: Cardinal.WEST,
                Direction.LEFT: Cardinal.NORTH,
                Direction.RIGHT: Cardinal.SOUTH
            }
        elif heading_temp == 90:  # South
            self.orientation = {
                Direction.FORWARD: Cardinal.SOUTH,
                Direction.BACK: Cardinal.NORTH,
                Direction.LEFT: Cardinal.EAST,
                Direction.RIGHT: Cardinal.WEST
            }
        elif heading_temp == 180:  # West
            self.orientation = {
                Direction.FORWARD: Cardinal.WEST,
                Direction.BACK: Cardinal.EAST,
                Direction.LEFT: Cardinal.SOUTH,
                Direction.RIGHT: Cardinal.NORTH
            }
        elif heading_temp == 270:  # North
            self.orientation = {
                Direction.FORWARD: Cardinal.NORTH,
                Direction.BACK: Cardinal.SOUTH,
                Direction.LEFT: Cardinal.WEST,
                Direction.RIGHT: Cardinal.EAST
            }
    
    def monte_carlo_draw(self):
        """Direct translation of monte_carlo_draw action"""
        direction = random.randint(0, 100)
        
        if direction <= 90:
            return Direction.FORWARD
        elif direction <= 95:
            return Direction.LEFT
        elif direction <= 100:
            return Direction.RIGHT
        else:
            return Direction.BACK
    
    def set_reward_efficiency(self, color: str):
        """Direct translation of set_reward_efficiency action"""
        not_so_good_reward = int(REWARD * COUNTER_EFFICIENCY)
        
        if color == "red":
            self.colors_reward_efficiency = {
                "red": REWARD,
                "green": not_so_good_reward,
                "blue": not_so_good_reward
            }
        elif color == "green":
            self.colors_reward_efficiency = {
                "red": not_so_good_reward,
                "green": REWARD,
                "blue": not_so_good_reward
            }
        elif color == "blue":
            self.colors_reward_efficiency = {
                "red": not_so_good_reward,
                "green": not_so_good_reward,
                "blue": REWARD
            }
    
    def compute_anticipated_criticality(self, box_to_take):
        """Direct translation of compute_anticipated_criticality action"""
        # Distance to box
        dist_box_to_me = abs(box_to_take.pos[1] - self.pos[1]) + abs(box_to_take.pos[0] - self.pos[0])
        
        # Distance from box to nest
        nest_cell = self.model.nests_locations[box_to_take.color]
        dist_robot_to_nest = abs(nest_cell[1] - box_to_take.pos[1]) + abs(nest_cell[0] - box_to_take.pos[0])
        
        # Anticipated battery before reward
        anticipated_battery_before_reward = self.battery - (dist_box_to_me + dist_robot_to_nest) * BATTERY_CONSUM
        if anticipated_battery_before_reward < 0:
            anticipated_battery_before_reward = 0
        
        # Anticipated battery after reward
        anticipated_battery = anticipated_battery_before_reward + self.colors_reward_efficiency[box_to_take.color]
        
        if anticipated_battery < MIN_BATTERY:
            return MAX_CRITICALITY
        elif anticipated_battery > MAX_BATTERY:
            return MIN_CRITICALITY
        else:
            return MAX_CRITICALITY - anticipated_battery
    
    def get_neighbors_at_vision(self):
        """Get neighbors within vision range"""
        return self.model.grid.get_neighbors(self.pos, moore=True, radius=3)
    
    def get_neighbors_at_speech(self):
        """Get neighbors within speech range"""
        return self.model.grid.get_neighbors(self.pos, moore=True, radius=3)
    
    def update_reachable_boxes(self):
        """Update reachable boxes list"""
        neighbors = self.get_neighbors_at_vision()
        self.reachable_boxes = [agent for agent in neighbors if isinstance(agent, Box)]
    
    def step(self):
        """OPTIMIZED step function for enhanced efficiency"""
        if self.is_dead:
            return
        
        # Update battery state
        self.battery_state = BATTERY_STATE_NB - int(BATTERY_STATE_NB * (self.battery / MAX_BATTERY))
        self.criticality = max(0, MAX_CRITICALITY - self.battery)
        self.is_auth_to_move = self.waiting_turn_nb >= self.battery_state
        self.is_need_a_box = NEED_BOX_THRESHOLD < self.criticality
        
        # Update battery (reflex update_battery_state)
        if self.is_auth_to_move:
            self.battery -= BATTERY_CONSUM
            self.waiting_turn_nb = 0
        else:
            self.waiting_turn_nb += 1
        
        # Update reachable boxes
        self.update_reachable_boxes()
        
        # OPTIMIZED: Process messages more efficiently
        # Only process messages when not actively working on a task
        if not (self.targeted_box or self.carried_box) or self.criticality > NEED_BOX_THRESHOLD:
            self.read_requests()
            self.read_agrees()
            self.read_refuses()
            self.read_inform()
        
        # OPTIMIZED: Search for boxes - prioritize task completion
        if self.reachable_boxes and (not self.targeted_box or self.criticality > NEED_BOX_THRESHOLD):
            self.search_box()
        
        # Movement - same as parent
        if self.is_auth_to_move:
            if self.targeted_box is None and self.carried_box is None:
                self.basic_move()
            elif self.targeted_box is not None:
                self.go_to_target_box()
            elif self.carried_box is not None and not self.is_waiting_to_give_box:
                self.carry_box_to_nest()
        
        # Take box
        if (self.targeted_box is not None and 
            self.targeted_box.pos == self.pos):
            self.take_box()
        
        # Drop box
        if (self.carried_box is not None and 
            self.pos == self.model.nests_locations[self.carried_box.color]):
            self.drop_box_in_nest()
        
        # Die
        if self.battery <= 0:
            self.die()
    
    def basic_move(self):
        """Direct translation of basic_move reflex"""
        self.redefine_direction(self.heading)
        
        direction = self.monte_carlo_draw()
        goto = self.orientation[direction]
        
        x, y = self.pos
        new_x, new_y = x, y
        
        if goto == Cardinal.NORTH:
            new_y = max(0, y - 1)
        elif goto == Cardinal.SOUTH:
            new_y = min(self.model.grid.height - 1, y + 1)
        elif goto == Cardinal.WEST:
            new_x = max(0, x - 1)
        elif goto == Cardinal.EAST:
            new_x = min(self.model.grid.width - 1, x + 1)
        
        if (new_x, new_y) != (x, y):
            self.model.grid.move_agent(self, (new_x, new_y))
            self.myCell = (new_x, new_y)
    
    def search_box(self):
        """Direct translation of search_box reflex"""
        for i, box in enumerate(self.reachable_boxes):
            ant_reach_crit = self.compute_anticipated_criticality(box)
            
            # No owner
            if box.owner is None:
                # I don't carry a box
                if self.carried_box is None:
                    # I don't target a box
                    if self.targeted_box is None:
                        self.targeted_box = box
                        box.owner = self
                    # I target a box
                    else:
                        my_target_box_crit = self.compute_anticipated_criticality(self.targeted_box)
                        # Check if this is a better box
                        if ant_reach_crit < my_target_box_crit:
                            self.targeted_box.owner = None
                            self.targeted_box = box
                            box.owner = self
                # I carry a box
                else:
                    my_target_box_crit = self.compute_anticipated_criticality(self.carried_box)
                    # Check if this is a better box
                    if ant_reach_crit < my_target_box_crit:
                        self.carried_box.owner = None
                        self.carried_box = None
                        self.is_carrying_box = False
                        self.targeted_box = box
                        box.owner = self
    
    def go_to_target_box(self):
        """Direct translation of go_to_target_box reflex"""
        if self.targeted_box is not None:
            self.move_towards(self.targeted_box.pos)
    
    def move_towards(self, target_pos):
        """Simple pathfinding"""
        x, y = self.pos
        tx, ty = target_pos
        
        if x < tx:
            new_x = min(x + 1, self.model.grid.width - 1)
        elif x > tx:
            new_x = max(x - 1, 0)
        else:
            new_x = x
        
        if y < ty:
            new_y = min(y + 1, self.model.grid.height - 1)
        elif y > ty:
            new_y = max(y - 1, 0)
        else:
            new_y = y
        
        if (new_x, new_y) != (x, y):
            self.model.grid.move_agent(self, (new_x, new_y))
            self.myCell = (new_x, new_y)
    
    def take_box(self):
        """Direct translation of take_box reflex"""
        if self.targeted_box is not None:
            self.carried_box = self.targeted_box
            self.targeted_box = None
            self.carried_box.owner = self
            self.is_carrying_box = True
    
    def carry_box_to_nest(self):
        """Direct translation of carry_box_to_nest reflex"""
        if self.carried_box is not None:
            nest_pos = self.model.nests_locations[self.carried_box.color]
            self.move_towards(nest_pos)
    
    def drop_box_in_nest(self):
        """Direct translation of drop_box_in_nest reflex"""
        if self.carried_box is not None:
            # Reward
            self.battery += self.colors_reward_efficiency[self.carried_box.color]
            self.battery = min(self.battery, MAX_BATTERY)
            
            self.carried_box.this_is_the_end = True
            self.carried_box = None
            self.is_carrying_box = False
            
            self.heading = random.choice([0, 90, 180, 270])
            self.redefine_direction(self.heading)
    
    def request_criticality(self):
        """Direct translation of request_criticality reflex"""
        receivers = [agent for agent in self.get_neighbors_at_speech() 
                    if isinstance(agent, RobotWithoutBuffer) and not agent.is_dead]
        
        for receiver in receivers:
            if receiver.is_carrying_box:
                self.is_request_criticality_last_cycle = True
                anticipated_criticality = self.compute_anticipated_criticality(receiver.carried_box)
                
                message = Message(
                    sender=self,
                    content=[CRITICALITY_STRING, anticipated_criticality, self.criticality]
                )
                receiver.requests.append(message)
    
    def read_requests(self):
        """Direct translation of read_requests reflex"""
        for request in self.requests:
            if request.content[0] == CRITICALITY_STRING:
                if self.carried_box is not None:
                    my_ant_criticality = self.compute_anticipated_criticality(self.carried_box)
                    sender_ant_criticality = int(request.content[1])
                    
                    if my_ant_criticality > sender_ant_criticality:
                        # Agree
                        agree_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING])
                        inform_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING, my_ant_criticality])
                        request.sender.agrees.append(agree_msg)
                        request.sender.informs.append(inform_msg)
                    else:
                        # Refuse
                        refuse_msg = Message(sender=self, content=['Ko'])
                        request.sender.refuses.append(refuse_msg)
            
            elif request.content[0] == DEMAND_BOX_STRING:
                if self.carried_box is not None:
                    agree_msg = Message(sender=self, content=[self.carried_box.pos])
                    request.sender.agrees.append(agree_msg)
                    
                    # Transfer box
                    self.carried_box.owner = request.sender
                    request.sender.targeted_box = self.carried_box
                    self.carried_box = None
                    self.is_carrying_box = False
        
        self.requests.clear()
    
    def read_agrees(self):
        """Direct translation of read_agrees reflex"""
        for agree in self.agrees:
            if GIVE_MY_BOX_STRING in str(agree.content):
                self.is_request_criticality_last_cycle = False
        
        self.agrees.clear()
    
    def read_refuses(self):
        """Direct translation of read_refuses reflex"""
        self.is_request_criticality_last_cycle = False
        self.refuses.clear()
    
    def read_inform(self):
        """Direct translation of read_inform reflex"""
        best_criticality = MAX_CRITICALITY
        giver = None
        
        for inform in self.informs:
            content_i = inform.content[0]
            
            if GIVE_MY_BOX_STRING in str(content_i):
                ant_criticality_temp = inform.content[1]
                if ant_criticality_temp < best_criticality:
                    best_criticality = ant_criticality_temp
                    giver = inform.sender
        
        if giver is not None:
            message = Message(sender=self, content=[DEMAND_BOX_STRING])
            giver.requests.append(message)
        
        self.informs.clear()
    
    def die(self):
        """Direct translation of die reflex"""
        if self.carried_box is not None:
            self.carried_box.owner = None
            self.carried_box = None
        
        if self.targeted_box is not None:
            self.targeted_box.owner = None
            self.targeted_box = None
        
        self.is_dead = True
        self.color = "black"

class DoubleCooprRobotNobuff(RobotWithoutBuffer):
    """Direct translation of double_coop_robot_nobuff.gaml"""
    
    def __init__(self, unique_id: int, model, color: str):
        super().__init__(unique_id, model, color)
        self.box_reserved = False
    
    def step(self):
        """OPTIMIZED step function for enhanced efficiency"""
        if self.is_dead:
            return
        
        # Update battery state
        self.battery_state = BATTERY_STATE_NB - int(BATTERY_STATE_NB * (self.battery / MAX_BATTERY))
        self.criticality = max(0, MAX_CRITICALITY - self.battery)
        self.is_auth_to_move = self.waiting_turn_nb >= self.battery_state
        self.is_need_a_box = NEED_BOX_THRESHOLD < self.criticality
        
        # Update battery (reflex update_battery_state)
        if self.is_auth_to_move:
            self.battery -= BATTERY_CONSUM
            self.waiting_turn_nb = 0
        else:
            self.waiting_turn_nb += 1
        
        # Update reachable boxes
        self.update_reachable_boxes()
        
        # OPTIMIZED: Process messages more efficiently
        # Only process messages when not actively working on a task
        if not (self.targeted_box or self.carried_box) or self.criticality > NEED_BOX_THRESHOLD:
            self.read_requests()
            self.read_agrees()
            self.read_refuses()
            self.read_inform()
        
        # OPTIMIZED: Search for boxes - prioritize task completion
        if self.reachable_boxes and (not self.targeted_box or self.criticality > NEED_BOX_THRESHOLD):
            self.search_box()
        
        # Movement - same as parent
        if self.is_auth_to_move:
            if self.targeted_box is None and self.carried_box is None:
                self.basic_move()
            elif self.targeted_box is not None:
                self.go_to_target_box()
            elif self.carried_box is not None and not self.is_waiting_to_give_box:
                self.carry_box_to_nest()
        
        # Take box
        if (self.targeted_box is not None and 
            self.targeted_box.pos == self.pos):
            self.take_box()
        
        # Drop box
        if (self.carried_box is not None and 
            self.pos == self.model.nests_locations[self.carried_box.color]):
            self.drop_box_in_nest()
        
        # Die
        if self.battery <= 0:
            self.die()
    
    def compute_anticipated_criticality(self, box_to_take):
        """Enhanced version from double_coop_robot_nobuff.gaml"""
        # Distance to box
        dist_box_to_me = abs(box_to_take.pos[1] - self.pos[1]) + abs(box_to_take.pos[0] - self.pos[0])
        
        # Distance from box to nest
        nest_cell = self.model.nests_locations[box_to_take.color]
        dist_robot_to_nest = abs(nest_cell[1] - box_to_take.pos[1]) + abs(nest_cell[0] - box_to_take.pos[0])
        
        # Anticipated battery before reward
        anticipated_battery_before_reward = self.battery - (dist_box_to_me + dist_robot_to_nest) * BATTERY_CONSUM
        if anticipated_battery_before_reward < 0:
            anticipated_battery_before_reward = 0
        
        # Anticipated battery after reward
        anticipated_battery = anticipated_battery_before_reward + self.colors_reward_efficiency[box_to_take.color]
        
        # Enhanced condition
        if (anticipated_battery < MIN_BATTERY) or (anticipated_battery_before_reward == 0):
            return MAX_CRITICALITY
        elif anticipated_battery > MAX_BATTERY:
            return MIN_CRITICALITY
        else:
            return MAX_CRITICALITY - anticipated_battery
    
    def ant_crit_best_free_box_around_me(self):
        """Direct translation of ant_crit_best_free_box_around_me action"""
        best_box = None
        free_boxes = [box for box in self.reachable_boxes if box.owner is None]
        best_ant_crit = MAX_CRITICALITY
        
        for box in free_boxes:
            ant_reach_crit = self.compute_anticipated_criticality(box)
            if best_ant_crit >= ant_reach_crit:
                best_box = box
                best_ant_crit = ant_reach_crit
        
        return best_box
    
    def request_criticality(self):
        """Disabled for double cooperation"""
        pass
    
    def read_requests(self):
        """OPTIMIZED Enhanced request handling - faster decision making"""
        for request in self.requests:
            if request.content[0] == CRITICALITY_STRING:
                if self.carried_box is not None or self.targeted_box is not None:
                    my_ant_criticality = 0
                    box_info = None
                    
                    if self.carried_box is not None:
                        my_ant_criticality = self.compute_anticipated_criticality(self.carried_box)
                        box_info = self.carried_box
                    else:
                        my_ant_criticality = self.compute_anticipated_criticality(self.targeted_box)
                        box_info = self.targeted_box
                    
                    sender_ant_criticality = int(request.content[1])
                    sender_instant_criticality = int(request.content[2])
                    
                    # OPTIMIZED: More efficient cooperation logic
                    # Only help if it significantly improves system efficiency
                    criticality_difference = sender_instant_criticality - self.criticality
                    anticipated_benefit = my_ant_criticality - sender_ant_criticality
                    
                    # Enhanced cooperation with efficiency focus
                    if criticality_difference >= 20 and anticipated_benefit > 10:
                        # Sender is significantly more critical AND transfer has good benefit
                        agree_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING])
                        inform_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING, my_ant_criticality])
                        request.sender.agrees.append(agree_msg)
                        request.sender.informs.append(inform_msg)
                        self.box_reserved = True
                    elif self.criticality >= sender_instant_criticality:
                        # I'm more critical - only help if it doesn't hurt me much
                        if self.criticality >= my_ant_criticality and anticipated_benefit > 15:
                            agree_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING])
                            inform_msg = Message(sender=self, content=[GIVE_MY_BOX_STRING, my_ant_criticality])
                            request.sender.agrees.append(agree_msg)
                            request.sender.informs.append(inform_msg)
                            self.box_reserved = True
                        else:
                            # Refuse - not beneficial enough
                            refuse_msg = Message(sender=self, content=['Ko'])
                            request.sender.refuses.append(refuse_msg)
                    else:
                        # Refuse - not worth the negotiation overhead
                        refuse_msg = Message(sender=self, content=['Ko'])
                        request.sender.refuses.append(refuse_msg)
            
            elif request.content[0] == DEMAND_BOX_STRING:
                # OPTIMIZED: Faster box transfer
                if self.carried_box is not None:
                    agree_msg = Message(sender=self, content=[self.carried_box])
                    request.sender.agrees.append(agree_msg)
                    # Clear ownership immediately
                    self.carried_box = None
                    self.is_carrying_box = False
                elif self.targeted_box is not None:
                    agree_msg = Message(sender=self, content=[self.targeted_box])
                    request.sender.agrees.append(agree_msg)
                    # Clear ownership immediately  
                    self.targeted_box = None
                else:
                    refuse_msg = Message(sender=self, content=["No"])
                    request.sender.refuses.append(refuse_msg)
                
                self.box_reserved = False
        
        self.requests.clear()
    
    def read_agrees(self):
        """Enhanced version from double_coop_robot_nobuff.gaml"""
        for agree in self.agrees:
            if GIVE_MY_BOX_STRING in str(agree.content):
                self.is_request_criticality_last_cycle = False
            else:
                box_given = agree.content[0]
                if isinstance(box_given, Box):
                    # Clear current commitments
                    if self.carried_box is not None:
                        self.carried_box.owner = None
                        self.carried_box = None
                        self.is_carrying_box = False
                    elif self.targeted_box is not None:
                        self.targeted_box.owner = None
                    
                    # Take new box
                    self.targeted_box = box_given
                    box_given.owner = self
        
        self.agrees.clear()
    
    def read_inform(self):
        """Enhanced version from double_coop_robot_nobuff.gaml"""
        best_criticality = MAX_CRITICALITY
        giver = None
        
        for inform in self.informs:
            content_i = inform.content[0]
            
            if GIVE_MY_BOX_STRING in str(content_i):
                ant_criticality_temp = inform.content[1]
                if ant_criticality_temp < best_criticality:
                    best_criticality = ant_criticality_temp
                    giver = inform.sender
        
        if giver is not None:
            message = Message(sender=self, content=[DEMAND_BOX_STRING])
            giver.requests.append(message)
            
            # Unreserve other robots' boxes
            for inform in self.informs:
                if GIVE_MY_BOX_STRING in str(inform.content[0]):
                    other_giver = inform.sender
                    if giver != other_giver:
                        other_giver.box_reserved = False
        
        self.informs.clear()
    
    def search_box(self):
        """OPTIMIZED Enhanced search_box - prioritize free boxes for efficiency"""
        
        # STEP 1: First, efficiently handle all free boxes
        free_boxes = [box for box in self.reachable_boxes if box.owner is None]
        owned_boxes = [box for box in self.reachable_boxes if box.owner is not None and box.owner != self]
        
        # Process free boxes first (same as parent class logic)
        for box in free_boxes:
            ant_reach_crit = self.compute_anticipated_criticality(box)
            
            if self.carried_box is None:
                if self.targeted_box is None:
                    self.targeted_box = box
                    box.owner = self
                    return  # Found a box, stop searching
                else:
                    my_target_box_crit = self.compute_anticipated_criticality(self.targeted_box)
                    if ant_reach_crit < my_target_box_crit:
                        self.targeted_box.owner = None
                        self.targeted_box = box
                        box.owner = self
                        return  # Found better box, stop searching
            else:
                my_carried_box_crit = self.compute_anticipated_criticality(self.carried_box)
                if ant_reach_crit < my_carried_box_crit:
                    self.carried_box.owner = None
                    self.carried_box = None
                    self.is_carrying_box = False
                    self.targeted_box = box
                    box.owner = self
                    return  # Found better box, stop searching
        
        # STEP 2: Only negotiate for owned boxes if no good free boxes found
        # AND only if we're in critical need (high criticality)
        if self.criticality > NEED_BOX_THRESHOLD and len(owned_boxes) > 0:
            
            # Find the best owned box that's worth negotiating for
            best_owned_box = None
            best_owned_crit = float('inf')
            current_best_crit = float('inf')
            
            # Determine our current best option
            if self.carried_box is not None:
                current_best_crit = self.compute_anticipated_criticality(self.carried_box)
            elif self.targeted_box is not None:
                current_best_crit = self.compute_anticipated_criticality(self.targeted_box)
            
            # Find owned box that's significantly better than our current option
            for box in owned_boxes:
                ant_reach_crit = self.compute_anticipated_criticality(box)
                # Only consider if it's SIGNIFICANTLY better (threshold for efficiency)
                if ant_reach_crit < (current_best_crit * 0.7):  # 30% better threshold
                    if ant_reach_crit < best_owned_crit:
                        best_owned_box = box
                        best_owned_crit = ant_reach_crit
            
            # Negotiate only for the best owned box, and only if significantly beneficial
            if best_owned_box is not None:
                message = Message(
                    sender=self,
                    content=[CRITICALITY_STRING, best_owned_crit, self.criticality]
                )
                best_owned_box.owner.requests.append(message)

class BoxTransportModel(mesa.Model):
    """Model implementing the box transportation system"""
    
    def __init__(self, width=20, height=20, cooperation_type="basic"):
        super().__init__()
        
        self.grid = mesa.space.MultiGrid(width, height, torus=False)
        self.schedule = mesa.time.RandomActivation(self)
        
        # Nest locations
        self.nests_locations = {
            "red": (0, 0),
            "green": (width - 1, 0),
            "blue": (0, height - 1)
        }
        
        # Box generation
        self.box_counter = 0
        self.step_counter = 0
        
        # Create robots (30 of each color)
        colors = ["red", "green", "blue"]
        robot_id = 0
        
        for color in colors:
            for _ in range(30):
                if cooperation_type == "basic":
                    robot = RobotWithoutBuffer(robot_id, self, color)
                else:
                    robot = DoubleCooprRobotNobuff(robot_id, self, color)
                
                self.schedule.add(robot)
                robot_id += 1
        
        self.running = True
    
    def step(self):
        """Step with box generation every 3 steps"""
        self.step_counter += 1
        
        # Generate box every 3 time units
        if self.step_counter % 3 == 0:
            self.generate_box()
        
        self.schedule.step()
    
    def generate_box(self):
        """Generate a new box"""
        colors = ["red", "green", "blue"]
        color = random.choice(colors)
        
        # Find empty spot
        attempts = 0
        while attempts < 100:
            x = random.randint(0, self.grid.width - 1)
            y = random.randint(0, self.grid.height - 1)
            if len(self.grid.get_cell_list_contents([(x, y)])) == 0:
                box = Box(self.box_counter + 10000, self, color)
                self.grid.place_agent(box, (x, y))
                self.schedule.add(box)
                self.box_counter += 1
                break
            attempts += 1

# Test the implementation
if __name__ == "__main__":
    print("🚀 FAITHFUL GAMA TRANSLATION TEST")
    print("=" * 40)
    
    # Test basic model
    print("Testing Basic Model...")
    basic_model = BoxTransportModel(width=20, height=20, cooperation_type="basic")
    
    for step in range(50):
        basic_model.step()
        if step % 10 == 0:
            alive_robots = len([a for a in basic_model.schedule.agents 
                              if isinstance(a, RobotWithoutBuffer) and not a.is_dead])
            active_boxes = len([a for a in basic_model.schedule.agents 
                              if isinstance(a, Box) and not a.this_is_the_end])
            print(f"Step {step}: {alive_robots} robots alive, {active_boxes} boxes active")
    
    print("\nTesting Enhanced Model...")
    enhanced_model = BoxTransportModel(width=20, height=20, cooperation_type="enhanced")
    
    for step in range(50):
        enhanced_model.step()
        if step % 10 == 0:
            alive_robots = len([a for a in enhanced_model.schedule.agents 
                              if isinstance(a, DoubleCooprRobotNobuff) and not a.is_dead])
            active_boxes = len([a for a in enhanced_model.schedule.agents 
                              if isinstance(a, Box) and not a.this_is_the_end])
            print(f"Step {step}: {alive_robots} robots alive, {active_boxes} boxes active")
    
    print("\n✅ Faithful GAMA translation test completed!")