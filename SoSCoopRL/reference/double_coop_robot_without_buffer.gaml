/**
 *  utils
 *  Author: Bouziat
 *  Description: 
 */

model double_coop_robot
import "../experiment_parent.gaml"
import "../box.gaml"
import "../grid.gaml"
import "../nest.gaml"
import "robot_without_buffer.gaml"


global {


	//int seed <- 0 parameter: true;

}

entities {
	
	
	
	species double_coop_robot_nobuff skills:[moving, communicating] parent: robot_without_buffer {
		
		list<box> reachable_boxes update: box inside (myCell.neighbours_at_robot_vision);
		bool box_reserved <- false;
		
		action compute_anticipated_criticality(box box_to_take) {
			//do write_cycle(string("box_to_take: " + box_to_take));
			int dist_box_to_me <- abs(box_to_take.myCell.grid_y - self.myCell.grid_y) + abs(box_to_take.myCell.grid_x - self.myCell.grid_x);
			//do write_cycle(string("dist_box_to_me: " + dist_box_to_me));
			cell nest_cell <- nests_locations[box_to_take.color];
			int dist_robot_to_nest <- abs(nest_cell.grid_y - box_to_take.myCell.grid_y) + abs(nest_cell.grid_x - box_to_take.myCell.grid_x);
			//do write_cycle(string("dist_robot_to_nest: " + dist_robot_to_nest));
			int anticipated_battery_before_reward <- battery - (dist_box_to_me + dist_robot_to_nest)*battery_consum;
			if anticipated_battery_before_reward < 0 {
				anticipated_battery_before_reward <- 0;
			}
			int anticipated_battery <- anticipated_battery_before_reward + colors_reward_efficiency[box_to_take.color];
			//do write_cycle(string("anticipated_battery: " + anticipated_battery));
			if (anticipated_battery < min_battery) or (anticipated_battery_before_reward = 0){
				return max_criticality;
			} else if (anticipated_battery > max_battery) {
				return min_criticality;
			} else {
//				if (carried_box != nil) {
//					carried_box.owner <- nil;
//					carried_box <- nil;
//					is_carrying_box <- false;
//				}
				return max_criticality - anticipated_battery;
			}
		}
		
		box ant_crit_best_free_box_around_me {
			box best_box <- nil;
			bool end_reachable_boxes <- false;
			list<box> free_boxes <- box where (each.owner = nil) inside myCell.neighbours_at_robot_vision;
			int best_ant_crit <- max_criticality;
			int i <- 0;
			if (!empty(free_boxes)) {
				loop while: !end_reachable_boxes {
					int box_efficiency <- colors_reward_efficiency[(free_boxes[i]).color];			
					//do write_cycle("" +free_boxes[i]);		
					int ant_reach_crit <- compute_anticipated_criticality(free_boxes[i]);
					if (best_ant_crit >= ant_reach_crit) {
						best_box <- free_boxes[i];
						best_ant_crit <- ant_reach_crit;
					}
					
					i <- i + 1;
					if ((reachable_boxes count true) = 1) {
						end_reachable_boxes <- true;
					} else {
						end_reachable_boxes <- (i = (free_boxes count true));
					}
				}
			}
			
			return best_box;
		}
		
		reflex request_criticality when: is_need_a_box and carried_box = nil and targeted_box = nil and !is_request_criticality_last_cycle and !is_dead{}
		
		reflex read_requests when: !is_dead and !empty(requests){
			loop request over: requests {
				switch request.content[0] {
					match criticality_string {
						
						//do inform with: [ message :: request, content :: [criticality_string + ':' + criticality] ];
						// calcul la criticité anticipée et décide si oui ou non il laisse sa boite
						if (carried_box != nil) or (targeted_box != nil) {
							int my_ant_criticality <- 0;
							if (carried_box != nil) {
								my_ant_criticality <- compute_anticipated_criticality(carried_box);
							} else {
								my_ant_criticality <- compute_anticipated_criticality(targeted_box);
							}
							int sender_ant_criticality <- int(request.content[1]);
							int sender_instant_criticality <- int(request.content[2]);
							box box_info <- (carried_box != nil) ? carried_box : targeted_box;
							// im the more critical, i decide
							if (self.criticality >= sender_instant_criticality) {
								//if im more critical than the sender, I check if the give of my box do not raise my crit
								if (self.criticality >= my_ant_criticality) {
									do refuse with: [ message :: request, content :: ['Ko'] ];
//									box best_free_box <- ant_crit_best_free_box_around_me();
//									// If i found a box which gives me a CA which is not worst than other robot CA, i give
//									if (best_free_box != nil) {
//										int best_box_ant_crit <- compute_anticipated_criticality(best_free_box);
//										if (best_box_ant_crit <= sender_ant_criticality) {
//											do agree with: [message :: request, content :: [give_my_box_string] ];
//											do inform with: [message :: request, content :: [give_my_box_string, my_ant_criticality] ];
//											box_reserved <- true;	
//										} else {
//											do refuse with: [ message :: request, content :: ['Ko'] ];
//										}
//									} else {
//										do refuse with: [ message :: request, content :: ['Ko'] ];
//									}
//									
									//do write_cycle('Dont propose my box('+box_info+') to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
								} else {
									do agree with: [message :: request, content :: [give_my_box_string] ];
									do inform with: [message :: request, content :: [give_my_box_string, my_ant_criticality] ];
									box_reserved <- true;
									//do write_cycle('Propose my box(' + box_info + ') ' + 'to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
								}
							// im not the most critical, i help the other robot, 
							} else if  (self.criticality + 10) <= sender_instant_criticality{
								//if (sender_instant_criticality < sender_ant_criticality) {
									do write_cycle("Send you my box because youre more critical");
									do agree with: [message :: request, content :: [give_my_box_string] ];
									do inform with: [message :: request, content :: [give_my_box_string, my_ant_criticality] ];
									box_reserved <- true;	
								//}
										
									//do write_cycle('Propose my box(' + box_info + ') ' + 'to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
							} else {
								do refuse with: [ message :: request, content :: ['Ko'] ];
							}
//							if (my_ant_criticality > sender_ant_criticality and !box_reserved) {
//								do agree with: [message :: request, content :: [give_my_box_string] ];
//								do inform with: [message :: request, content :: [give_my_box_string, my_ant_criticality] ];
//								box_reserved <- true;
//								do write_cycle('Propose my box(' + box_info + ') ' + 'to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
//							} else {
//								do refuse with: [ message :: request, content :: ['Ko'] ];
//								do write_cycle('Dont propose my box('+box_info+') to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
//							}
						}
					}
					match demand_box_string {

						
						if (carried_box != nil) {
							do agree with: [ message :: request, content :: [carried_box] ];
							//do write_cycle('Give my box to ' + request.sender + ' ' + carried_box);
						} else if (targeted_box != nil){
							do agree with: [ message :: request, content :: [targeted_box] ];
							//do write_cycle('Give my box to ' + request.sender + ' ' + targeted_box);
						} else {
							do refuse with: [ message :: request, content :: ["No"] ];
							//do write_cycle('Refuse ' + request.sender + ' ' + targeted_box);
						}
						
						//do inform with: [ message :: request, content :: ['waiting'] ];
						double_coop_robot_nobuff robott <- (request.sender as double_coop_robot_nobuff);
//						if (carried_box != nil) {
//							if (robott.carried_box = nil){
//								carried_box.owner <- robott;
//							}
//							robott.targeted_box <- carried_box;
//						} else if (targeted_box != nil) {
//							if (robott.carried_box = nil){
//								targeted_box.owner <- robott;
//							}
//							robott.targeted_box <- targeted_box;
//						}
						


						carried_box <- nil;
						targeted_box <- nil;
						is_carrying_box <- false;
						box_reserved <- false;
					}
				}
			}
		}
		
		reflex read_agrees when: !empty(agrees) and !is_dead{
			
			loop agree over: agrees {
				if (agree.content contains give_my_box_string) {
					is_request_criticality_last_cycle <- false;
					//do write_cycle('Agent ' + agree.sender + ' says Ok for giving');
				} else {
					box box_given <- (agree.content[0] as box);
					//do write_cycle('Info:' + agree.sender + ' gives ' + box_given);
					//location <- box_given.location;
					if (carried_box != nil) {
						carried_box.owner <- nil;
						carried_box <- nil;
						is_carrying_box <- false;
					} else if (targeted_box != nil) {
						targeted_box.owner <- nil;
					}
					targeted_box <- box_given;
					targeted_box.owner <- self;
					//do write_cycle('Agent ' + agree.sender + ' gives me his box('+ targeted_box +') at ' + location);
					
					////do write_cycle('Agent ' + agree.sender + ' gives me his box at ' + location);
				}
			}
		}
		
		reflex read_inform when: !empty(informs) and !is_dead{
			int best_criticality <- max_criticality;
			double_coop_robot_nobuff giver <- nil;
			//do write_cycle("Receive inform");
			loop inform over: informs {
				string content_i <- inform.content[0];
				
				if (content_i contains give_my_box_string) {
					//list<string> content_list <- content_i split_with ":";
					int ant_criticality_temp <- inform.content[1];
					string s_message <- 'Receive criticality of ' + inform.sender + "(" + ant_criticality_temp + ") ";
					string s_critic <- 'best(' + best_criticality + ')';  
					//do write_cycle(s_message + s_critic);
					if (ant_criticality_temp < best_criticality) {
						best_criticality <- ant_criticality_temp;
						giver <- (inform.sender as double_coop_robot_nobuff);
					}
				}
			}
			
			if (giver != nil) {
				do send with: [ receivers :: giver, protocol :: 'fipa-request', performative :: 'request', content :: [demand_box_string] ];
				//do write_cycle('Request box of ' + giver);
				//Enlever la reservation des autres
				loop inform over: informs {
					string content_i <- inform.content[0];
					double_coop_robot_nobuff other_giver <- (inform.sender as double_coop_robot_nobuff);
					if (content_i contains give_my_box_string) {
						if (giver != other_giver) {
							//do write_cycle("Unreserved " + other_giver);
							other_giver.box_reserved <- false;
						}
					}
				}
			}
			
			
		}
		
		reflex compute_criticality {
			
		}
		
		reflex search_box when: !empty(reachable_boxes) and !is_dead{
			//do write_cycle('serach box begin :'+cell_boxes_buffer);
			bool end_reachable_boxes <- false;
			int i <- 0;

			////do write_cycle(string(reachable_boxes));
			// Try to get a box of my color
			loop while: !end_reachable_boxes {
				bool message_sent <- false;
				int box_efficiency <- colors_reward_efficiency[(reachable_boxes[i]).color];			
				//do write_cycle("i=" +i);		
				int ant_reach_crit <- compute_anticipated_criticality((reachable_boxes[i]));
				//No owner
				if ((reachable_boxes[i]).owner = nil) {
					// I dont carry a box
					if (carried_box = nil) {
						// I dont target a box
						if (targeted_box = nil) {
							targeted_box <- reachable_boxes[i];
							targeted_box.owner <- self;
						// I target a box
						} else {
							int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
							// I check if this is a better box
							if (ant_reach_crit < my_target_box_crit) {
								targeted_box.owner <- nil;
								targeted_box <- reachable_boxes[i];
								targeted_box.owner <- self;
							}
						}
					// I carry a box
					} else {
						int my_target_box_crit <- compute_anticipated_criticality(carried_box);
						// I check if this is a better box
						if (ant_reach_crit < my_target_box_crit) {
							carried_box.owner <- nil;
							carried_box <- nil;
							is_carrying_box <- false;
							targeted_box <- reachable_boxes[i];
							targeted_box.owner <- self;
						}
					}
				// Box has a owner
				} else {
					// I check if im not the owner
					if ((reachable_boxes[i]).owner != self and !message_sent) {
						// I dont carry a box
						if (carried_box = nil) {
							// I dont target a box
							if (targeted_box = nil) {
								//Send a message for the box
								do send with: [ receivers :: (reachable_boxes[i]).owner, protocol :: 'fipa-request', performative :: 'request', content :: [criticality_string,ant_reach_crit, self.criticality] ];
							} else {
								int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
								// I check if this is a better box
								if (ant_reach_crit < my_target_box_crit) {
									//Send a message for the box
									do send with: [ receivers :: (reachable_boxes[i]).owner, protocol :: 'fipa-request', performative :: 'request', content :: [criticality_string,ant_reach_crit, self.criticality] ];
								}
							}
					 	// I carry a box
						} else {
							int my_target_box_crit <- compute_anticipated_criticality(carried_box);
							// I check if this is a better box
							if (ant_reach_crit < my_target_box_crit) {
								do send with: [ receivers :: (reachable_boxes[i]).owner, protocol :: 'fipa-request', performative :: 'request', content :: [criticality_string,ant_reach_crit, self.criticality] ];
							}
						}
					}
				}
				i <- i + 1;
				if ((reachable_boxes count true) = 1) {
					end_reachable_boxes <- true;
				} else {
					end_reachable_boxes <- (i = (reachable_boxes count true));
				}

						
			}

		}
		


	}

}







