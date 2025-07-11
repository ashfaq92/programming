/**
* Name: robot_random 
* Author: ashfa
* Tags: CoCaRo
*/



model robot_cooperative

import "robot_base.gaml"

species robot_cooperative parent: robot_base skills: [fipa] {
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
		float dist_box_to_me_bk <- self distance_to box_to_take;
		nest nest_cell <- nest first_with (each.color = box_to_take.color);
		int dist_robot_to_nest <- abs(nest_cell.myCell.grid_x - self.myCell.grid_x) + abs(nest_cell.myCell.grid_y - self.myCell.grid_y);
		int anticipated_battery_before_reward <- battery - (dist_box_to_me + dist_robot_to_nest) * battery_consum;
		
		if anticipated_battery_before_reward < 0 {
			anticipated_battery_before_reward <- 0;
		}
				
		int anticipated_battery <- anticipated_battery_before_reward + colors_reward_efficiency(box_to_take.color);
		
		if anticipated_battery < min_battery {
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
    		int box_efficiency <- colors_reward_efficiency(bx.color);
    		int ant_reach_crit <- compute_anticipated_criticality(bx);  // ant. crit. of reachable box
    		
    		
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
    	}
    }
	
	// ===== COOPERATION WITH OTHER ROBOTS =====
	reflex request_criticality when: is_need_a_box and carried_box=nil and targeted_box=nil and !is_request_criticality_last_cycle and battery>0 {		
		list<robot_cooperative> receivers <- (robot_cooperative where (myCell.neighbors_at_robot_speech contains each.myCell)) where (each.battery > 0);
				
		loop receiver over: receivers {
			if receiver.carried_box != nil {
				is_request_criticality_last_cycle <- true;
				write 'i request box to:' + receiver;
				int anticipated_criticality <- compute_anticipated_criticality(receiver.carried_box);
				do start_conversation to: [receiver] protocol:'fipa-request' performative: 'request' contents: [criticality_string, anticipated_criticality, self.criticality];
			}
		}
	}
	
	reflex read_requests when: !empty(requests) and battery > 0 {
		write 'reading requests';
		loop request over: requests {
			string request_type <- request.contents[0];
			if request_type=criticality_string and carried_box != nil {
				int my_ant_crit <- compute_anticipated_criticality(carried_box);
				int sender_ant_crit <- int(request.contents[1]);
				
				if (my_ant_crit > sender_ant_crit) {
	                //do agree with: [message :: request, content :: [give_my_box_string]];
	                do agree message: request contents: [give_my_box_string];
	                //do inform with: [message :: request, content :: [give_my_box_string, my_crit]];
	                do inform message: request contents:[give_my_box_string, my_ant_crit];
	                write ('Propose my box to ' + request.sender);
	            } else {
	                //do refuse with: [message :: request, content :: ['Ko']];
	            	do refuse message: request contents: ['Ko'] ;
	            	write ('refused request. as: ' + 'my_ant_crit' + string(my_ant_crit) + ' sender_ant_crit' + string(sender_ant_crit));
	            }
			}
			
			if (request_type=demand_box_string and carried_box != nil) {  // Add carried_box != nil check
			//if (request_type=demand_box_string) {
				
				// potential issues: dropping receiver's box, comparing criticality before exchange
				
	            robot_cooperative receiver <- (request.sender as robot_cooperative);
	            				
	            do agree message: request contents: [carried_box.location];
	            do inform message: request contents: ['waiting'];
	            	            
	            carried_box.owner <- receiver;
	            receiver.targeted_box <- carried_box;
	            carried_box <- nil;
	            write('----box exchange done------');
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
			 	point tempVar <- (agree.contents) as point;  
			 	write tempVar;
			 	location <- tempVar;
			 }
		}
	}
	
	reflex read_refuses when:!empty(refuses) and battery > 0 {
		is_request_criticality_last_cycle <- false;
	}
	
	reflex read_inform when: !empty(informs) and battery > 0 {
		int best_criticality <- max_criticality;
		robot_cooperative giver <- nil;
		loop inform over: informs {
			/*For example: 
			* inform: message[sender: robot_cooperative46; receivers: [robot_cooperative63]; performative: inform; content: [GiveMyBoxToYou, 300]; content]
			* inform.contents:  ['GiveMyBoxToYou',300]
			*/
			string content_type <- inform.contents[0];
			if content_type=give_my_box_string {
				int ant_criticality_temp <- int(inform.contents[1]);
				
				if ant_criticality_temp < best_criticality {
					best_criticality <- ant_criticality_temp;
					giver <- (inform.sender as robot_cooperative);
				}
			}
		}
	    // request the box from the best giver
	    if (giver != nil) {
    	    // Check if giver is still in communication range
    	    // list<robot_cooperative> nearby <- (robot_cooperative where (myCell.neighbors_at_robot_speech contains each.myCell));
	    	// if (nearby contains giver) {
	    	write 'requesting box from best giver: ' + giver;
	    	do start_conversation to: [giver] protocol: 'fipa-request' performative: 'request' contents: [demand_box_string];	    	
	    }	
	}
} // end of robot_cooperative species
