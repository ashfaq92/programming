/**
* Name: robot_random 
* Author: ashfa
* Tags: CoCaRo
*/



model robot_double_cooperative

import "robot_base.gaml"

species robot_double_cooperative parent: robot_base skills: [fipa] {
    // ==DOUBLE-COOP RELATED===
    bool box_reserved <- false;
    
    // ===== ATTRIBUTES =====
    int need_box_threshold <- int(max_criticality / 2);
	bool is_need_a_box update: need_box_threshold < criticality;
    int criticality <- 0 update: max_criticality - battery max: max_criticality;
    bool is_request_criticality_last_cycle <- false;    
    
    string criticality_string <- "criticality";
    string demand_box_string <- "GiveMeYourBox";
    string give_my_box_string <- "GiveMyBoxToYou";
	
	
	
	// ===== Calculate ANTICIPATED Criticality =====
	int compute_anticipated_criticality (box_ box_to_take) {
		int dist_box_to_me <- abs(box_to_take.myCell.grid_x - self.myCell.grid_x) + abs(box_to_take.myCell.grid_y - self.myCell.grid_y);
		nest nest_cell <- nest first_with (each.color = box_to_take.color);
		int dist_robot_to_nest <- abs(nest_cell.myCell.grid_x - self.myCell.grid_x) + abs(nest_cell.myCell.grid_y - self.myCell.grid_y);
		int anticipated_battery_before_reward <- battery - (dist_box_to_me + dist_robot_to_nest) * battery_consum;
		
		if anticipated_battery_before_reward < 0 {
			anticipated_battery_before_reward <- 0;
		}
				
		int anticipated_battery <- anticipated_battery_before_reward + colors_reward_efficiency(box_to_take.color);
		
		if anticipated_battery < min_battery or anticipated_battery_before_reward = 0 {
			return max_criticality;
		} else if anticipated_battery > max_battery {
			return min_criticality;
		} else {
			return max_criticality - anticipated_battery;
		}
	}
	
    // ===== MOVE & SEARCH ===    
    reflex search_box when: !empty(reachable_boxes) and battery > 0 {
    	loop bx over: reachable_boxes {
    		bool message_sent <- false;
    		int box_efficiency <- colors_reward_efficiency(bx.color);
    		int ant_reach_crit <- compute_anticipated_criticality(bx);  // ant. crit. of reachable box
    		
    		if bx.owner = nil {
    			if carried_box = nil {
	    			if targeted_box = nil {		// dont carry dont target 
	    				targeted_box <- bx;
	    				targeted_box.owner <- self;
	    			} else {					// dont carry but target
	    				int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
	    				// check if this is a better box
	    				if ant_reach_crit < my_target_box_crit {
	    					write 'better box!! suppress target';
	    					targeted_box.owner <- nil;
	    					targeted_box <- bx;
	    					targeted_box.owner <- self;
	    				}
	    			}
	    		} else {	// i carry a box
	    			int my_carried_box_crit <- compute_anticipated_criticality(carried_box);
	    			// check if this is a better box
	    			if ant_reach_crit < my_carried_box_crit {
	    				write 'better box!! suppress carried';
	    				carried_box.owner <-nil;		//verify
	    				carried_box <- nil;
	    				targeted_box <- bx;
	    				targeted_box.owner <- self;
	    			}
	    		}
    		// box has an owner	
    		} else {
    			// check if i am not the owner
    			if bx.owner != self and !message_sent {   // problem 'message_sent' is always false
    				//  i dont carry  a box
    				if carried_box = nil {
    					// i dont target a box
    					if targeted_box = nil {
				    		do start_conversation to: [bx.owner] protocol: 'fipa-request' performative: 'request' contents: [criticality_string, ant_reach_crit, self.criticality];
    					} else {
    						int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
    						// i check if this is a better box
    						if ant_reach_crit < my_target_box_crit {
    							do start_conversation to: [bx.owner] protocol: 'fipa-request' performative: 'request' contents: [criticality_string, ant_reach_crit, self.criticality];
    						}
    					}
    				// i carry a box
    				} else {
    					int my_target_box_crit <- compute_anticipated_criticality(carried_box);
    					// i check if this is a better box
    					if ant_reach_crit < my_target_box_crit {
    						do start_conversation to: [bx.owner] protocol: 'fipa-request' performative: 'request' contents: [criticality_string, ant_reach_crit, self.criticality];
    					}
    				}
    			}
    			
    		}
			
    	}
    }
	
	// ===== COOPERATION WITH OTHER ROBOTS =====
	reflex request_criticality when: is_need_a_box and carried_box=nil and targeted_box=nil and !is_request_criticality_last_cycle and battery>0 {}
	
	reflex read_requests when: !empty(requests) and battery > 0 {
		write 'reading requests';
		loop request over: requests {
			string request_type <- request.contents[0];
			if request_type=criticality_string and (carried_box != nil or targeted_box != nil) {
				int my_ant_crit <- 0;
				
				if carried_box != nil {
					my_ant_crit <- compute_anticipated_criticality(carried_box);
				} else {
					my_ant_crit <- compute_anticipated_criticality(targeted_box);
				}
				
				int sender_ant_crit <- int(request.contents[1]);
				int sender_instant_crit <- int(request.contents[2]);  // CHECK!!
				//write('request contentst are: ' + request.contents);
				//write('request contentst at index 2 are: ' + request.contents[2]);
				
				if (self.criticality >= sender_instant_crit) {
					if (self.criticality >= my_ant_crit) {
						do refuse message: request contents: ['Ko'] ;
					} else {
						do agree message: request contents: [give_my_box_string];
						do inform message: request contents:[give_my_box_string, my_ant_crit];
						box_reserved <- true;
					}
				} else if (self.criticality + 10) <= sender_instant_crit {
					write ("sending you box bcz you're more critical");
					do agree message: request contents: [give_my_box_string];
					do inform message: request contents:[give_my_box_string, my_ant_crit];
					box_reserved <- true;
				} else {
					do refuse message: request contents: ['Ko'] ;
				}
			}
			
			if (request_type=demand_box_string) {
				if carried_box != nil {
					do agree message: request contents: [carried_box];	
				}  else if targeted_box != nil {
					do agree message: request contents: [targeted_box];
				} else {
					do refuse message: request contents: ["No"];
				}
									            	            
				// potential issues: dropping receiver's box, comparing criticality before exchange
	            carried_box <- nil;
	            targeted_box <- nil;
	            box_reserved <- false;
	        }
		}
	}
	
	reflex read_agrees when:!empty(agrees) and battery > 0 {
		loop agree over: agrees {	
			write agree.contents;
			string agree_type <- agree.contents[0];
			 
			 if agree_type=give_my_box_string {
			 	is_request_criticality_last_cycle <- false;
			 	// write 'Agent ' + agree.sender + ' says ok for giving';
			 } else {
			 	box_ box_given <- (agree.contents[0] as box_);
			 	
			 	if carried_box != nil {
			 		carried_box.owner <- nil;
			 		carried_box <- nil;
			 	} else if targeted_box != nil {
			 		targeted_box.owner <- nil;
			 	}
			 	
			 	targeted_box <- box_given;
			 	targeted_box.owner <- self;
			 }
		}
	}
	
	reflex read_refuses when:!empty(refuses) and battery > 0 {
		is_request_criticality_last_cycle <- false;
	}
	
	reflex read_inform when: !empty(informs) and battery > 0 {
		int best_criticality <- max_criticality;
		robot_double_cooperative giver <- nil;
		loop inform over: informs {
			/*For example: 
			* inform: message[sender: robot_double_cooperative46; receivers: [robot_double_cooperative63]; performative: inform; content: [GiveMyBoxToYou, 300]; content]
			* inform.contents:  ['GiveMyBoxToYou',300]
			*/
			string content_type <- inform.contents[0];
			if content_type=give_my_box_string {
				int ant_criticality_temp <- int(inform.contents[1]);
				
				if ant_criticality_temp < best_criticality {
					best_criticality <- ant_criticality_temp;
					giver <- (inform.sender as robot_double_cooperative);
				}
			}
		}
	    // request the box from the best giver
	    if (giver != nil) {
    	    // Check if giver is still in communication range
    	    // list<robot_double_cooperative> nearby <- (robot_double_cooperative where (myCell.neighbors_at_robot_speech contains each.myCell));
	    	// if (nearby contains giver) {
	    	write 'requesting box from best giver: ' + giver;
	    	do start_conversation to: [giver] protocol: 'fipa-request' performative: 'request' contents: [demand_box_string];
	    	
	    	loop inform over: informs {
	    		string content_type <- inform.contents[0];
	    		robot_double_cooperative other_giver <- (inform.sender as robot_double_cooperative);
	    		if content_type = give_my_box_string and giver != other_giver {
	    			other_giver.box_reserved <- false;
	    		}
	    	}   	
	    }	
	}
} // end of robot_double_cooperative species






