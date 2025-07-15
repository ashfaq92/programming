import mesa
import random
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional


class RobotType(Enum):
    RANDOM = "random"
    GREEDY = "greedy"
    COOPERATIVE = "cooperative"
    SAPHESIA = "saphesia"


class BoxAgent(mesa.Agent):
    """Simple box - direct translation from GAMA"""

    def __init__(self, unique_id: int, model, color: str = None):
        super().__init__(unique_id, model)

        # Direct GAMA translation: color <- one_of(colors);
        if color is None:
            colors = ["red", "green", "blue"]
            self.color = random.choice(colors)
        else:
            self.color = color

        # Direct GAMA translation: agent owner <- nil;
        self.owner = None

        # For Mesa compatibility
        self.is_carried = False
        self.carrier = None



class NestAgent(mesa.Agent):
    """Direct GAMA translation"""

    def __init__(self, unique_id: int, model, color: str):
        super().__init__(unique_id, model)

        # Direct GAMA translation: rgb color;
        self.color = color

        # GAMA: cell myCell <- cell closest_to self;
        # Mesa handles this automatically with self.pos
        self.deposited_boxes = 0  # Track deliveries



class RobotAgent(mesa.Agent):
    """Direct translation of GAMA robot_base"""

    def __init__(self, unique_id: int, model, color: str, robot_type: RobotType):
        super().__init__(unique_id, model)

        # Direct GAMA translation: COMMON ATTRIBUTES
        self.color = color
        self.robot_type = robot_type

        self.max_battery = 300
        self.min_battery = 0
        self.initial_battery = self.max_battery
        self.battery_consum = 1
        self.battery = self.initial_battery  # min: min_battery max: max_battery

        self.reward = int((2 * self.max_battery) / 3)  # 200
        self.reduced_reward = int(0.2 * self.reward)  # 40

        self.min_criticality = self.min_battery
        self.max_criticality = self.max_battery

        # Movement and state
        self.previous_location = None
        self.speed = 1.0
        self.robot_speed = 1  # Will be set properly in init

        # Targeting
        self.target_nest = None
        self.carried_box = None
        self.targeted_box = None

        # Vision (GAMA: reachable_boxes update: (box_ where (myCell.neighbors_at_robot_vision contains each.myCell)))
        self.vision_range = 3  # From cell robot_vision_range

        # GAMA: cell myCell update: cell closest_to self
        # Mesa handles this with self.pos automatically

    def colors_reward_efficiency(self, box_color: str) -> int:
        """Direct GAMA translation"""
        if box_color == self.color:
            return self.reward
        else:
            return self.reduced_reward

    def compute_anticipated_criticality(self, box_to_take) -> int:
        """Direct GAMA translation of complex criticality calculation"""
        # GAMA: int dist_box_to_me <- abs(box_to_take.myCell.grid_x - self.myCell.grid_x) + abs(box_to_take.myCell.grid_y - self.myCell.grid_y);
        dist_box_to_me = abs(box_to_take.pos[0] - self.pos[0]) + abs(box_to_take.pos[1] - self.pos[1])

        # GAMA: nest nest_cell <- nest first_with (each.color = box_to_take.color);
        nest_cell = None
        for agent in self.model.schedule.agents:
            if isinstance(agent, NestAgent) and agent.color == box_to_take.color:
                nest_cell = agent
                break

        if not nest_cell:
            return self.max_criticality

        # GAMA: int dist_robot_to_nest <- abs(nest_cell.myCell.grid_x - self.myCell.grid_x) + abs(nest_cell.myCell.grid_y - self.myCell.grid_y);
        dist_robot_to_nest = abs(nest_cell.pos[0] - self.pos[0]) + abs(nest_cell.pos[1] - self.pos[1])

        # GAMA: int anticipated_battery_before_reward <- battery - (dist_box_to_me + dist_robot_to_nest) * battery_consum;
        anticipated_battery_before_reward = self.battery - (dist_box_to_me + dist_robot_to_nest) * self.battery_consum

        # GAMA: if anticipated_battery_before_reward < 0 { anticipated_battery_before_reward <- 0; }
        if anticipated_battery_before_reward < 0:
            anticipated_battery_before_reward = 0

        # GAMA: int anticipated_battery <- anticipated_battery_before_reward + colors_reward_efficiency(box_to_take.color);
        anticipated_battery = anticipated_battery_before_reward + self.colors_reward_efficiency(box_to_take.color)

        # GAMA: if anticipated_battery < min_battery or anticipated_battery_before_reward = 0 { return max_criticality; }
        if anticipated_battery < self.min_battery or anticipated_battery_before_reward == 0:
            return self.max_criticality
        # GAMA: else if anticipated_battery > max_battery { return min_criticality; }
        elif anticipated_battery > self.max_battery:
            return self.min_criticality
        # GAMA: else { return max_criticality - anticipated_battery; }
        else:
            return self.max_criticality - anticipated_battery

    def get_reachable_boxes(self):
        """GAMA: reachable_boxes update: (box_ where (myCell.neighbors_at_robot_vision contains each.myCell))"""
        reachable = []
        for agent in self.model.schedule.agents:
            if isinstance(agent, BoxAgent) and not agent.is_carried:
                distance = abs(agent.pos[0] - self.pos[0]) + abs(agent.pos[1] - self.pos[1])
                if distance <= self.vision_range:
                    reachable.append(agent)
        return reachable

    def update_battery(self):
        """GAMA: reflex update_battery when: battery > 0"""
        if self.battery > 0:
            # GAMA: bool moved_step <- (location != previous_location);
            moved_step = (self.pos != self.previous_location) if self.previous_location else False
            if moved_step:
                # GAMA: battery <- battery - battery_consum;
                self.battery -= self.battery_consum
            self.previous_location = self.pos

    def basic_move(self):
        """GAMA: reflex basic_move when: targeted_box=nil and carried_box=nil and battery > 0"""
        if self.targeted_box is None and self.carried_box is None and self.battery > 0:
            # GAMA: do wander amplitude: 30.0;
            self.wander()

    def wander(self):
        """Simple wandering movement"""
        possible_moves = self.model.grid.get_neighborhood(
            self.pos, moore=False, include_center=False
        )
        if possible_moves:
            new_pos = random.choice(possible_moves)
            if not self.model.grid.out_of_bounds(new_pos):
                self.model.grid.move_agent(self, new_pos)

    def step(self):
        """Main step function - translates GAMA reflexes"""

        # GAMA: reflex update_battery when: battery > 0
        self.update_battery()

        # GAMA: reflex die when: battery <= 0
        if self.battery <= 0:
            if self.carried_box is not None:
                # GAMA: carried_box.owner <- nil; carried_box <- nil;
                self.carried_box.owner = None
                self.carried_box = None
            if self.targeted_box is not None:
                # GAMA: targeted_box.owner <- nil; targeted_box <- nil;
                self.targeted_box.owner = None
                self.targeted_box = None
            # GAMA: color <- rgb("black"); // dont kill robot to prevent survivorship bias
            self.color = "black"
            return  # Stop processing when dead

        # GAMA: reflex carry_box_to_nest when: carried_box != nil and battery > 0
        if self.carried_box is not None and self.battery > 0:
            # Check if box still exists (GAMA: if (!dead(carried_box)))
            if self.carried_box in self.model.boxes:
                # GAMA: target_nest <- nest first_with (each.color = carried_box.color);
                if self.target_nest is None:
                    for agent in self.model.schedule.agents:
                        if isinstance(agent, NestAgent) and agent.color == self.carried_box.color:
                            self.target_nest = agent
                            break

                # Move toward nest
                if self.target_nest:
                    self.move_towards(self.target_nest.pos)
            else:
                # GAMA: else { carried_box <- nil; }
                self.carried_box = None

        # GAMA: reflex update_carried_box_position when: carried_box != nil and battery > 0
        if self.carried_box is not None and self.battery > 0:
            # GAMA: carried_box.myCell <- myCell; carried_box.location <- location;
            self.model.grid.move_agent(self.carried_box, self.pos)

        # GAMA: reflex drop_box_in_nest when: carried_box != nil and target_nest != nil and battery > 0 and !dead(carried_box)
        if (self.carried_box is not None and self.target_nest is not None and
                self.battery > 0 and self.carried_box in self.model.boxes):

            # GAMA: if myCell = carried_box.myCell and myCell = target_nest.myCell and carried_box.myCell = target_nest.myCell
            if (self.pos == self.carried_box.pos and self.pos == self.target_nest.pos and
                    self.carried_box.pos == self.target_nest.pos):

                # GAMA: battery <- battery + colors_reward_efficiency(carried_box.color);
                self.battery += self.colors_reward_efficiency(self.carried_box.color)

                # GAMA: box_ box_to_remove <- carried_box;
                box_to_remove = self.carried_box

                # GAMA: ask agents { if targeted_box = box_to_remove { targeted_box <- nil; } if carried_box = box_to_remove { carried_box <- nil; } }
                for robot in self.model.robots:
                    if robot.targeted_box == box_to_remove:
                        robot.targeted_box = None
                    if robot.carried_box == box_to_remove:
                        robot.carried_box = None

                # GAMA: ask box_to_remove { do die; }
                if box_to_remove in self.model.boxes:
                    self.model.boxes.remove(box_to_remove)
                self.model.grid.remove_agent(box_to_remove)
                self.model.schedule.remove(box_to_remove)

                # GAMA: carried_box <- nil; target_nest <- nil;
                self.carried_box = None
                self.target_nest = None

        # GAMA: reflex take_box when: targeted_box!=nil and targeted_box.myCell=myCell and battery > 0
        if (self.targeted_box is not None and self.targeted_box in self.model.boxes and
                self.targeted_box.pos == self.pos and self.battery > 0):
            # GAMA: carried_box <- targeted_box; targeted_box <- nil;
            self.carried_box = self.targeted_box
            self.carried_box.is_carried = True
            self.carried_box.carrier = self
            self.targeted_box = None

        # GAMA: reflex go_to_target_box when: targeted_box != nil and battery > 0
        elif self.targeted_box is not None and self.battery > 0:
            if self.targeted_box in self.model.boxes:
                self.move_towards(self.targeted_box.pos)
            else:
                self.targeted_box = None  # Box no longer exists

        # GAMA: reflex basic_move when: targeted_box=nil and carried_box=nil and battery > 0
        else:
            self.basic_move()

    def move_towards(self, target_pos):
        """Move one step towards target"""
        x, y = self.pos
        tx, ty = target_pos

        # Simple movement towards target
        if x < tx:
            x += 1
        elif x > tx:
            x -= 1
        elif y < ty:
            y += 1
        elif y > ty:
            y -= 1

        new_pos = (x, y)
        if not self.model.grid.out_of_bounds(new_pos):
            self.model.grid.move_agent(self, new_pos)


class GreedyRobotAgent(RobotAgent):
    """Direct translation of GAMA robot_greedy"""

    def search_box(self):
        """GAMA: reflex search_box when: !empty(reachable_boxes) and battery > 0"""
        reachable_boxes = self.get_reachable_boxes()

        if len(reachable_boxes) > 0 and self.battery > 0:
            # GAMA: loop bx over: reachable_boxes
            for bx in reachable_boxes:
                # GAMA: int ant_reach_crit <- compute_anticipated_criticality(bx);
                ant_reach_crit = self.compute_anticipated_criticality(bx)

                # GAMA: if carried_box = nil
                if self.carried_box is None:
                    # GAMA: if targeted_box = nil { ... }
                    if self.targeted_box is None:
                        # GAMA: targeted_box <- bx; targeted_box.owner <- self;
                        self.targeted_box = bx
                        self.targeted_box.owner = self
                    else:
                        # GAMA: not carrying but targeting
                        # GAMA: int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
                        my_target_box_crit = self.compute_anticipated_criticality(self.targeted_box)

                        # GAMA: if ant_reach_crit < my_target_box_crit
                        if ant_reach_crit < my_target_box_crit:
                            # GAMA: targeted_box.owner <- nil; targeted_box <- bx; targeted_box.owner <- self;
                            self.targeted_box.owner = None
                            self.targeted_box = bx
                            self.targeted_box.owner = self

                else:
                    # GAMA: i carry a box
                    # GAMA: int my_carried_box_crit <- compute_anticipated_criticality(carried_box);
                    my_carried_box_crit = self.compute_anticipated_criticality(self.carried_box)

                    # GAMA: if ant_reach_crit < my_carried_box_crit
                    if ant_reach_crit < my_carried_box_crit:
                        # GAMA: drop the current box
                        # GAMA: carried_box.owner <-nil; carried_box <- nil;
                        self.carried_box.owner = None
                        self.carried_box.is_carried = False
                        self.carried_box.carrier = None
                        self.carried_box = None

                        # GAMA: Target the new better box
                        # GAMA: targeted_box <- bx; targeted_box.owner <- self;
                        self.targeted_box = bx
                        self.targeted_box.owner = self

    def step(self):
        """Override to add greedy search behavior"""
        # Call parent step first (all the base robot behavior)
        super().step()

        # Then add greedy-specific behavior
        # GAMA: reflex search_box when: !empty(reachable_boxes) and battery > 0
        self.search_box()


class CooperativeRobotAgent(RobotAgent):
    """Direct translation of GAMA robot_double_cooperative"""

    def __init__(self, unique_id: int, model, color: str, robot_type: RobotType):
        super().__init__(unique_id, model, color, robot_type)

        # GAMA: === SAPHESIA RELATED ===
        self.helped_colors = []  # list<rgb> helped_colors <- [];

        # GAMA: ==DOUBLE-COOP RELATED===
        self.box_reserved = False  # bool box_reserved <- false;

        # GAMA: ===== ATTRIBUTES =====
        self.need_box_threshold = int(self.max_criticality / 2)  # int need_box_threshold <- int(max_criticality / 2);
        self.criticality = 0  # int criticality <- 0 update: max_criticality - battery max: max_criticality;
        self.is_request_criticality_last_cycle = False  # bool is_request_criticality_last_cycle <- false;

        # GAMA: Message strings
        self.criticality_string = "criticality"
        self.demand_box_string = "GiveMeYourBox"
        self.give_my_box_string = "GiveMyBoxToYou"

        # For Mesa communication (replacing FIPA)
        self.pending_requests = []
        self.pending_agrees = []
        self.pending_refuses = []
        self.pending_informs = []

    def update_criticality(self):
        """GAMA: criticality <- 0 update: max_criticality - battery max: max_criticality;"""
        self.criticality = min(self.max_criticality - self.battery, self.max_criticality)

    def is_need_a_box(self):
        """GAMA: bool is_need_a_box update: need_box_threshold < criticality;"""
        return self.need_box_threshold < self.criticality

    def colors_reward_efficiency(self, box_color: str) -> int:
        """GAMA: Override colors_reward_efficiency to support helping"""
        # GAMA: if (box_color = color or (helped_colors contains box_color)) { return reward; } else { return reduced_reward; }
        if box_color == self.color or box_color in self.helped_colors:
            return self.reward
        else:
            return self.reduced_reward

    def send_request(self, target_robot, message_type: str, contents: list):
        """Replace FIPA messaging with direct communication"""
        if hasattr(target_robot, 'pending_requests'):
            target_robot.pending_requests.append({
                'sender': self,
                'type': message_type,
                'contents': contents
            })

    def send_agree(self, target_robot, contents: list):
        """Replace FIPA agree with direct communication"""
        if hasattr(target_robot, 'pending_agrees'):
            target_robot.pending_agrees.append({
                'sender': self,
                'contents': contents
            })

    def send_refuse(self, target_robot, contents: list):
        """Replace FIPA refuse with direct communication"""
        if hasattr(target_robot, 'pending_refuses'):
            target_robot.pending_refuses.append({
                'sender': self,
                'contents': contents
            })

    def send_inform(self, target_robot, contents: list):
        """Replace FIPA inform with direct communication"""
        if hasattr(target_robot, 'pending_informs'):
            target_robot.pending_informs.append({
                'sender': self,
                'contents': contents
            })

    def search_box(self):
        """GAMA: reflex search_box when: !empty(reachable_boxes) and battery > 0"""
        reachable_boxes = self.get_reachable_boxes()

        if len(reachable_boxes) > 0 and self.battery > 0:
            # GAMA: loop bx over: reachable_boxes
            for bx in reachable_boxes:
                message_sent = False  # bool message_sent <- false;
                # GAMA: int ant_reach_crit <- compute_anticipated_criticality(bx);
                ant_reach_crit = self.compute_anticipated_criticality(bx)

                # GAMA: if bx.owner = nil
                if bx.owner is None:
                    # GAMA: if carried_box = nil
                    if self.carried_box is None:
                        # GAMA: if targeted_box = nil
                        if self.targeted_box is None:
                            # GAMA: targeted_box <- bx; targeted_box.owner <- self;
                            self.targeted_box = bx
                            self.targeted_box.owner = self
                        else:
                            # GAMA: int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
                            my_target_box_crit = self.compute_anticipated_criticality(self.targeted_box)
                            # GAMA: if ant_reach_crit < my_target_box_crit
                            if ant_reach_crit < my_target_box_crit:
                                # GAMA: targeted_box.owner <- nil; targeted_box <- bx; targeted_box.owner <- self;
                                self.targeted_box.owner = None
                                self.targeted_box = bx
                                self.targeted_box.owner = self
                    else:
                        # GAMA: i carry a box
                        # GAMA: int my_carried_box_crit <- compute_anticipated_criticality(carried_box);
                        my_carried_box_crit = self.compute_anticipated_criticality(self.carried_box)
                        # GAMA: if ant_reach_crit < my_carried_box_crit
                        if ant_reach_crit < my_carried_box_crit:
                            # GAMA: drop the current box
                            # GAMA: carried_box.owner <-nil; carried_box <- nil;
                            self.carried_box.owner = None
                            self.carried_box.is_carried = False
                            self.carried_box.carrier = None
                            self.carried_box = None
                            # GAMA: target the new better box
                            # GAMA: targeted_box <- bx; targeted_box.owner <- self;
                            self.targeted_box = bx
                            self.targeted_box.owner = self

                # GAMA: box has an owner
                else:
                    # GAMA: if bx.owner != self and !message_sent
                    if bx.owner != self and not message_sent:
                        # GAMA: i dont carry a box
                        if self.carried_box is None:
                            # GAMA: i dont target a box
                            if self.targeted_box is None:
                                # GAMA: do start_conversation to: [bx.owner] protocol: 'fipa-request' performative: 'request' contents: [criticality_string, ant_reach_crit, self.criticality];
                                self.send_request(bx.owner, 'request',
                                                  [self.criticality_string, ant_reach_crit, self.criticality])
                            else:
                                # GAMA: int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
                                my_target_box_crit = self.compute_anticipated_criticality(self.targeted_box)
                                # GAMA: if ant_reach_crit < my_target_box_crit
                                if ant_reach_crit < my_target_box_crit:
                                    self.send_request(bx.owner, 'request',
                                                      [self.criticality_string, ant_reach_crit, self.criticality])
                        # GAMA: i carry a box
                        else:
                            # GAMA: int my_target_box_crit <- compute_anticipated_criticality(carried_box);
                            my_target_box_crit = self.compute_anticipated_criticality(self.carried_box)
                            # GAMA: if ant_reach_crit < my_target_box_crit
                            if ant_reach_crit < my_target_box_crit:
                                self.send_request(bx.owner, 'request',
                                                  [self.criticality_string, ant_reach_crit, self.criticality])

    def read_requests(self):
        """GAMA: reflex read_requests when: !empty(requests) and battery > 0"""
        if len(self.pending_requests) > 0 and self.battery > 0:
            # GAMA: loop request over: requests
            for request in self.pending_requests:
                # GAMA: string request_type <- request.contents[0];
                request_type = request['contents'][0]

                # GAMA: if request_type=criticality_string and (carried_box != nil or targeted_box != nil)
                if request_type == self.criticality_string and (
                        self.carried_box is not None or self.targeted_box is not None):
                    my_ant_crit = 0

                    # GAMA: if carried_box != nil { my_ant_crit <- compute_anticipated_criticality(carried_box); } else { my_ant_crit <- compute_anticipated_criticality(targeted_box); }
                    if self.carried_box is not None:
                        my_ant_crit = self.compute_anticipated_criticality(self.carried_box)
                    else:
                        my_ant_crit = self.compute_anticipated_criticality(self.targeted_box)

                    # GAMA: int sender_ant_crit <- int(request.contents[1]); int sender_instant_crit <- int(request.contents[2]);
                    sender_ant_crit = int(request['contents'][1])
                    sender_instant_crit = int(request['contents'][2])

                    # GAMA: Complex decision logic
                    if self.criticality >= sender_instant_crit:
                        if self.criticality >= my_ant_crit:
                            # GAMA: do refuse message: request contents: ['Ko'] ;
                            self.send_refuse(request['sender'], ['Ko'])
                        else:
                            # GAMA: do agree message: request contents: [give_my_box_string]; do inform message: request contents:[give_my_box_string, my_ant_crit]; box_reserved <- true;
                            self.send_agree(request['sender'], [self.give_my_box_string])
                            self.send_inform(request['sender'], [self.give_my_box_string, my_ant_crit])
                            self.box_reserved = True
                    elif (self.criticality + 10) <= sender_instant_crit:
                        # GAMA: write ("sending you box bcz you're more critical");
                        print(f"Robot {self.unique_id}: sending box because requester is more critical")
                        self.send_agree(request['sender'], [self.give_my_box_string])
                        self.send_inform(request['sender'], [self.give_my_box_string, my_ant_crit])
                        self.box_reserved = True
                    else:
                        # GAMA: do refuse message: request contents: ['Ko'] ;
                        self.send_refuse(request['sender'], ['Ko'])

                # GAMA: if (request_type=demand_box_string)
                if request_type == self.demand_box_string:
                    # GAMA: if carried_box != nil { do agree message: request contents: [carried_box]; } else if targeted_box != nil { do agree message: request contents: [targeted_box]; } else { do refuse message: request contents: ["No"]; }
                    if self.carried_box is not None:
                        self.send_agree(request['sender'], [self.carried_box])
                    elif self.targeted_box is not None:
                        self.send_agree(request['sender'], [self.targeted_box])
                    else:
                        self.send_refuse(request['sender'], ["No"])

                    # GAMA: carried_box <- nil; targeted_box <- nil; box_reserved <- false;
                    self.carried_box = None
                    self.targeted_box = None
                    self.box_reserved = False

            # Clear processed requests
            self.pending_requests = []

    def read_agrees(self):
        """GAMA: reflex read_agrees when:!empty(agrees) and battery > 0"""
        if len(self.pending_agrees) > 0 and self.battery > 0:
            # GAMA: loop agree over: agrees
            for agree in self.pending_agrees:
                # GAMA: string agree_type <- agree.contents[0];
                agree_type = agree['contents'][0]

                # GAMA: if agree_type=give_my_box_string
                if agree_type == self.give_my_box_string:
                    # GAMA: is_request_criticality_last_cycle <- false;
                    self.is_request_criticality_last_cycle = False
                else:
                    # GAMA: box_ box_given <- (agree.contents[0] as box_);
                    box_given = agree['contents'][0]

                    # GAMA: if carried_box != nil { carried_box.owner <- nil; carried_box <- nil; } else if targeted_box != nil { targeted_box.owner <- nil; }
                    if self.carried_box is not None:
                        self.carried_box.owner = None
                        self.carried_box = None
                    elif self.targeted_box is not None:
                        self.targeted_box.owner = None

                    # GAMA: targeted_box <- box_given; targeted_box.owner <- self;
                    self.targeted_box = box_given
                    self.targeted_box.owner = self

            # Clear processed agrees
            self.pending_agrees = []

    def read_refuses(self):
        """GAMA: reflex read_refuses when:!empty(refuses) and battery > 0"""
        if len(self.pending_refuses) > 0 and self.battery > 0:
            # GAMA: is_request_criticality_last_cycle <- false;
            self.is_request_criticality_last_cycle = False
            # Clear processed refuses
            self.pending_refuses = []

    def read_inform(self):
        """GAMA: reflex read_inform when: !empty(informs) and battery > 0"""
        if len(self.pending_informs) > 0 and self.battery > 0:
            # GAMA: int best_criticality <- max_criticality; robot_double_cooperative giver <- nil;
            best_criticality = self.max_criticality
            giver = None

            # GAMA: loop inform over: informs
            for inform in self.pending_informs:
                # GAMA: string content_type <- inform.contents[0];
                content_type = inform['contents'][0]

                # GAMA: if content_type=give_my_box_string
                if content_type == self.give_my_box_string:
                    # GAMA: int ant_criticality_temp <- int(inform.contents[1]);
                    ant_criticality_temp = int(inform['contents'][1])

                    # GAMA: if ant_criticality_temp < best_criticality
                    if ant_criticality_temp < best_criticality:
                        best_criticality = ant_criticality_temp
                        giver = inform['sender']

            # GAMA: if (giver != nil)
            if giver is not None:
                # GAMA: do start_conversation to: [giver] protocol: 'fipa-request' performative: 'request' contents: [demand_box_string];
                self.send_request(giver, 'request', [self.demand_box_string])

                # GAMA: Reset other givers' reservations
                for inform in self.pending_informs:
                    content_type = inform['contents'][0]
                    other_giver = inform['sender']
                    if content_type == self.give_my_box_string and giver != other_giver:
                        if hasattr(other_giver, 'box_reserved'):
                            other_giver.box_reserved = False

            # Clear processed informs
            self.pending_informs = []

    def step(self):
        """Override to add cooperative behavior"""
        # Update criticality first
        self.update_criticality()

        # Call parent step (all base robot behavior)
        super().step()

        # Then add cooperative-specific behavior
        if self.battery > 0:
            # GAMA: reflex search_box when: !empty(reachable_boxes) and battery > 0
            self.search_box()

            # Process all pending communications
            self.read_requests()
            self.read_agrees()
            self.read_refuses()
            self.read_inform()

class RobotSimulation(mesa.Model):
    """Main simulation model"""

    def __init__(self, width=50, height=50, robot_type="cooperative", n_robots_per_color=30):
        super().__init__()

        self.width = width
        self.height = height
        self.robot_type = RobotType(robot_type)

        # Create grid and scheduler
        self.grid = mesa.space.MultiGrid(width, height, torus=False)
        self.schedule = mesa.time.RandomActivation(self)

        # Track agents
        self.robots = []
        self.boxes = []
        self.nests = []

        # ID management - keep track of next available ID
        self.next_id = 0

        # SApHESIA global state
        self.dying_threshold = 1.0 / 3.0
        self.help_threshold = 0.3



        # Fix the datacollector in RobotSimulation.__init__()
        self.datacollector = mesa.DataCollector(
            model_reporters={
                "Alive_Robots": lambda m: len([r for r in m.robots if r.battery > 0]),  # GAMA: battery > 0
                "Mean_Battery": lambda m: np.mean([r.battery for r in m.robots]) if m.robots else 0,
                # Include all robots like GAMA
                "Total_Boxes": lambda m: len([b for b in m.boxes if not b.is_carried]),
                "Red_Delivered": lambda m: sum(n.deposited_boxes for n in m.nests if n.color == "red"),
                "Green_Delivered": lambda m: sum(n.deposited_boxes for n in m.nests if n.color == "green"),
                "Blue_Delivered": lambda m: sum(n.deposited_boxes for n in m.nests if n.color == "blue"),
            }
        )

        self.setup_environment()
        self.running = True

    def setup_environment(self):
        """Create nests, robots, and initial boxes"""

        # Create nests at fixed positions
        nest_positions = [(15, 15), (35, 15), (25, 32)]
        colors = ["red", "green", "blue"]

        for pos, color in zip(nest_positions, colors):
            nest = NestAgent(self.next_id, self, color)
            self.schedule.add(nest)
            self.grid.place_agent(nest, pos)
            self.nests.append(nest)
            self.next_id += 1

        # Create robots
        for color in colors:
            for _ in range(30):  # 30 robots per color
                robot = RobotAgent(self.next_id, self, color, self.robot_type)

                # Find empty position
                attempts = 0
                while attempts < 100:
                    pos = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
                    if self.grid.is_cell_empty(pos):
                        break
                    attempts += 1

                self.schedule.add(robot)
                self.grid.place_agent(robot, pos)
                self.robots.append(robot)
                self.next_id += 1

        # Create initial boxes
        for _ in range(15):
            self.spawn_box()

    def spawn_box(self):
        """Spawn a random colored box at random location"""
        color = random.choice(["red", "green", "blue"])

        # Find empty position
        attempts = 0
        while attempts < 100:
            pos = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
            if self.grid.is_cell_empty(pos):
                box = BoxAgent(self.next_id, self, color)
                self.schedule.add(box)
                self.grid.place_agent(box, pos)
                self.boxes.append(box)
                self.next_id += 1
                return
            attempts += 1

        # If we couldn't find an empty spot, just don't spawn the box
        print(f"Warning: Could not find empty position to spawn box")

    def update_saphesia_cooperation(self):
        """Update SApHESIA cooperation logic (if robot_type is SAPHESIA)"""
        if self.robot_type != RobotType.SAPHESIA:
            return

        # Calculate dying ratios for each color
        color_stats = {"red": {"total": 0, "dying": 0},
                       "green": {"total": 0, "dying": 0},
                       "blue": {"total": 0, "dying": 0}}

        for robot in self.robots:
            if robot.is_alive:
                color_stats[robot.color]["total"] += 1
                if robot.battery < robot.max_battery * self.dying_threshold:
                    color_stats[robot.color]["dying"] += 1

        # Determine which colors need help
        colors_needing_help = []
        for color, stats in color_stats.items():
            if stats["total"] > 0:
                dying_ratio = stats["dying"] / stats["total"]
                if dying_ratio > self.help_threshold:
                    colors_needing_help.append(color)

        # Update robot cooperation
        for robot in self.robots:
            robot.helped_colors.clear()
            for color in colors_needing_help:
                if robot.color != color:
                    robot.helped_colors.add(color)

    def create_robots_with_colors(self, robot_type: str):
        """GAMA: action create_robots_with_colors(string robot_type)"""
        colors = ["red", "green", "blue"]

        for color in colors:
            for _ in range(30):  # 30 robots per color like GAMA
                if robot_type == "random":
                    robot = RandomRobotAgent(self.next_id, self, color, RobotType.RANDOM)
                elif robot_type == "greedy":
                    robot = GreedyRobotAgent(self.next_id, self, color, RobotType.GREEDY)
                elif robot_type == "cooperative":
                    robot = CooperativeRobotAgent(self.next_id, self, color, RobotType.COOPERATIVE)
                else:
                    robot = RobotAgent(self.next_id, self, color, RobotType.RANDOM)

                # Find empty position like GAMA
                attempts = 0
                while attempts < 100:
                    pos = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
                    if self.grid.is_cell_empty(pos):
                        break
                    attempts += 1

                self.schedule.add(robot)
                self.grid.place_agent(robot, pos)
                self.robots.append(robot)
                self.next_id += 1

    # In RobotSimulation.step() method, change:
    def step(self):
        """Run one step of the simulation"""
        # Update SApHESIA cooperation (skip for now)
        # self.update_saphesia_cooperation()

        # Spawn new boxes occasionally - GAMA: every 3 cycles
        if self.schedule.steps % 3 == 0:
            self.spawn_box()

        # Step all agents
        self.schedule.step()

        # Collect data
        self.datacollector.collect(self)



