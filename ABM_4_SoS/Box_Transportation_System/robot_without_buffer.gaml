/**
 *  utils
 *  Author: Bouziat
 *  Description: 
 */

model utils
import "../experiment_parent.gaml"
import "../box.gaml"
import "../grid.gaml"
import "../nest.gaml"


global {


	//int seed <- 0 parameter: true;
	
	const max_battery type: int <- 300;
	const reward type: int <- int((2*max_battery)/3);
	const counter_efficiency type: float <- 0.2;
	const min_battery type: int <- 0;
	const max_criticality type : int <- max_battery;
	const min_criticality type : int <- min_battery;
	const initial_battery type: int <- max_battery;
	const battery_consum type: int <- 1;

	const need_box_treshold type: int <- max_criticality/2;
	const battery_state_nb type: int <- 3;

	const max_boxes_buffer type: int <- 2;
	
	
	const criticality_string type: string <- 'criticality';
	const demand_box_string type: string <- 'givemeyourbox';
	const give_my_box_string type: string <- 'givemyboxtoyou';
}

entities {
	
	
	
	species robot_without_buffer skills: [moving, communicating]{
		
		const size type: float <- 1.0 ;
		int battery <- initial_battery min: min_battery max: max_battery; //update: battery - battery_consum min: min_battery max: max_battery ;
		rgb color;
		int robot_speed <- 0;
		//const efficiency type: map <- ['red'::rnd (10) / 10 , 'green'::rnd (10) / 10, 'blue'::rnd (10) / 10];
		cell myCell;
		list<box> reachable_boxes update: box inside (myCell.neighbours_at_robot_vision);
		box targeted_box <- nil;
		box carried_box <- nil;
		bool is_carrying_box <- false;
		bool is_empty_battery <- false;
		bool is_waiting_to_give_box <- false;
		bool is_need_a_box update: need_box_treshold < criticality;
		bool is_request_criticality_last_cycle <- false;
		bool is_diry_orientation <- false;
		bool is_auth_to_move update: waiting_turn_nb >= battery_state;
		bool is_dead <-false;
		//bool has_requests update: !empty(requests);
		// Baterry state:
		// 0: High ()
		// 1: Medium ()
		// 2: Low ()
		int battery_state <- (battery_state_nb - 1) min:0 max: (battery_state_nb - 1) update: battery_state_nb - battery_state_nb*(battery/max_battery);
		int waiting_turn_nb <- 0;
		int criticality <- 0 update: max_criticality - battery max: max_criticality;
		map<int, string> orientation <- nil;
		map<rgb, int> colors_reward_efficiency <- nil;
		/////////////////////////// USEFUL FONCTION /////////////////////////////////////////////////////////////////////
		action redefine_direction(map<int, string> orientation, int heading_temp) {
			switch heading_temp {
				// robot oriented to the east
				match 0 { 
					orientation[DIRECTION_ENUM.FORWARD] <-  CARDINAL_ENUM.EAST;
					orientation[DIRECTION_ENUM.BACK] <-  CARDINAL_ENUM.WEST;
					orientation[DIRECTION_ENUM.LEFT] <-  CARDINAL_ENUM.NORTH;
					orientation[DIRECTION_ENUM.RIGHT] <-  CARDINAL_ENUM.SOUTH;
				}
				// robot oriented to the south
				match 90 {
					orientation[DIRECTION_ENUM.FORWARD] <-  CARDINAL_ENUM.SOUTH;
					orientation[DIRECTION_ENUM.BACK] <-  CARDINAL_ENUM.NORTH;
					orientation[DIRECTION_ENUM.LEFT] <-  CARDINAL_ENUM.EAST;
					orientation[DIRECTION_ENUM.RIGHT] <-  CARDINAL_ENUM.WEST;
				}
				// robot oriented to the west
				match 180 {
					orientation[DIRECTION_ENUM.FORWARD] <-  CARDINAL_ENUM.WEST;
					orientation[DIRECTION_ENUM.BACK] <-  CARDINAL_ENUM.EAST;
					orientation[DIRECTION_ENUM.LEFT] <-  CARDINAL_ENUM.SOUTH;
					orientation[DIRECTION_ENUM.RIGHT] <-  CARDINAL_ENUM.NORTH;
				}
				// robot oriented to the north
				match 270 {
					orientation[DIRECTION_ENUM.FORWARD] <-  CARDINAL_ENUM.NORTH;
					orientation[DIRECTION_ENUM.BACK] <-  CARDINAL_ENUM.SOUTH;
					orientation[DIRECTION_ENUM.LEFT] <-  CARDINAL_ENUM.WEST;
					orientation[DIRECTION_ENUM.RIGHT] <-  CARDINAL_ENUM.EAST;
				}
			}
		}
		
		action monte_carlo_draw {
			int direction <- rnd(100);

			// go forward
			if  direction <= 90 {
				return DIRECTION_ENUM.FORWARD;
			// go left
			} else if direction <= 95 {
				return DIRECTION_ENUM.LEFT;
			//go right
			} else if direction <= 100 {
				return DIRECTION_ENUM.RIGHT;
			// go back
			} else {
				//return DIRECTION_ENUM.BACK;
			}
		}
		
		action set_reward_efficiency(rgb color) {
			
			int not_so_good_reward <- int(reward*counter_efficiency);
//			do write_cycle("" +max_battery);
//			do write_cycle("" +int((2*max_battery)/3));
//			do write_cycle("" +not_so_good_reward);
//			do write_cycle("" +reward);
			switch color {
				match rgb("red") {
					colors_reward_efficiency[rgb("red")] <- reward;
					colors_reward_efficiency[rgb("green")] <- not_so_good_reward;
					colors_reward_efficiency[rgb("blue")] <- not_so_good_reward;
				}
				match rgb("green") {
					colors_reward_efficiency[rgb("red")] <- not_so_good_reward;
					colors_reward_efficiency[rgb("green")] <- reward;
					colors_reward_efficiency[rgb("blue")] <- not_so_good_reward;
				}
				match rgb("blue") {
					colors_reward_efficiency[rgb("red")] <- not_so_good_reward;
					colors_reward_efficiency[rgb("green")] <- not_so_good_reward;
					colors_reward_efficiency[rgb("blue")] <- reward;
				}
			}
		}
		
		action write_cycle(string message) {
			//string cycle <- 'cycle ' + cycle + ',' + self + ': ';
			//write cycle + message;
		}
		
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
			if (anticipated_battery < min_battery) {
				return max_criticality;
			} else if (anticipated_battery > max_battery) {
				return min_criticality;
			} else {
				return max_criticality - anticipated_battery;
			}
		}
		
		list erase_first(list list_to_erase) {
			int i <- 0;
			//int length_list <- length(list_to_erase);
			int length_list <- list_to_erase count true;
			
			if (length_list > 1) {
				pair to_keep <- list_to_erase[length_list  - 1];
				loop times: (length_list - 1) {
					list_to_erase[i] <- list_to_erase[i+1];
					i <- i + 1;
				}
				//do write_cycle("before erase:"+list_to_erase);
				//do write_cycle("to erase:" + list_to_erase[length_list  - 1]);
				list_to_erase <- list_to_erase - list_to_erase[length_list  - 1];
				list_to_erase <- list_to_erase + to_keep;
			} else {
				list_to_erase <- [];
			}
			
			return list_to_erase;
			//do write_cycle("after erase:"+list_to_erase);
		}
		/////////////////////////// USEFUL FONCTION /////////////////////////////////////////////////////////////////////
		/// Voir le cas = 000000000 (quand c'est vide)
		
		init {
			heading <- shuffle([0, 90, 180, 270])[0];
			do redefine_direction(orientation, heading);
//			write "Initial heading:" + heading;
//			write "Initial orientation:";
//			write "orientation[DIRECTION_ENUM.FORWARD]:" + orientation[DIRECTION_ENUM.FORWARD];
//			write "orientation[DIRECTION_ENUM.LEFT]:" + orientation[DIRECTION_ENUM.LEFT];
//			write "orientation[DIRECTION_ENUM.RIGHT]:" + orientation[DIRECTION_ENUM.RIGHT];
//			write "orientation[DIRECTION_ENUM.BACK]:" + orientation[DIRECTION_ENUM.BACK];
            //Choose your color
//            int what_color <- rnd(100000);
//            if (what_color < 33333) {
//            	color <- rgb('red');
//            } else if (what_color < 66666) {
//            	color <- rgb('green');
//            } else {
//            	color <- rgb('blue');
//            }
            do set_reward_efficiency(color);
			myCell <- one_of (cell);
			list<robot_without_buffer> robots <- robot_without_buffer inside myCell;
			location <- myCell.location;
			loop while: (! empty(robots)) {
				myCell <- one_of (cell);
				robots <- robot_without_buffer inside myCell;
				location <- myCell.location;
			}
			cell cell0 <- cell[0,0];
			cell cell1 <- cell[1,0];
			robot_speed <- cell1.location.x - cell0.location.x;
		}
		
		reflex update_battery_state when: !is_dead {
			
			
//			if battery < max_battery/4 {
//				battery_state <- 2;
//			} else if (battery < 1*max_battery/2) {
//				battery_state <- 1;
//			} else {
//				battery_state <- 0;
//			}
			
			if (is_auth_to_move) {
				battery <- battery - battery_consum;
			}
			
		}
		
		
		reflex info when: !is_dead {
			//write "battery_state:" + battery_state + ";" + battery;
			//write "waiting_turn_nb:" + waiting_turn_nb;
			//write "is_auth_to_move:" + is_auth_to_move;
		}
		
		reflex count_waiting_turn when: !is_auth_to_move and !is_dead{
			waiting_turn_nb <- waiting_turn_nb + 1;
		}
		
		reflex basic_move when: targeted_box = nil and carried_box = nil and is_auth_to_move and !is_dead{
			waiting_turn_nb <-0;
			// Monte-carlo draw
			do redefine_direction(orientation, heading);
//			write "Current heading:" + heading;
//			write "Current orientation:";
//			write "orientation[DIRECTION_ENUM.FORWARD]:" + orientation[DIRECTION_ENUM.FORWARD];
//			write "orientation[DIRECTION_ENUM.LEFT]:" + orientation[DIRECTION_ENUM.LEFT];
//			write "orientation[DIRECTION_ENUM.RIGHT]:" + orientation[DIRECTION_ENUM.RIGHT];
//			write "orientation[DIRECTION_ENUM.BACK]:" + orientation[DIRECTION_ENUM.BACK];
	
			cell temp_cell <- nil;
			loop while: temp_cell = nil {
				string goto <- orientation[monte_carlo_draw()];
				switch goto {
					// robot oriented to the east
					match CARDINAL_ENUM.NORTH { 
						temp_cell <- cell[myCell.grid_x, myCell.grid_y - 1];
					}
					// robot oriented to the south
					match CARDINAL_ENUM.SOUTH {
						temp_cell <- cell[myCell.grid_x, myCell.grid_y + 1];
					}
					// robot oriented to the west
					match CARDINAL_ENUM.WEST {
						temp_cell <- cell[myCell.grid_x - 1, myCell.grid_y];
					}
					// robot oriented to the north
					match CARDINAL_ENUM.EAST {
						temp_cell <- cell[myCell.grid_x + 1, myCell.grid_y];
					}
				}
			}
			myCell <- temp_cell;
//			write "New position:" + "(" + myCell.grid_x + "," + myCell.grid_y + ")";
			location <- myCell.location;
		}
		
		reflex search_box when: !empty(reachable_boxes) and !is_dead{
			bool end_reachable_boxes <- false;
			int i <- 0;
			loop while: !end_reachable_boxes {
				bool message_sent <- false;
				int box_efficiency <- colors_reward_efficiency[(reachable_boxes[i]).color];			
				//do write_cycle("i=" +i);		
				int ant_reach_crit <- compute_anticipated_criticality((reachable_boxes[i]));
				//No owner
				if ((reachable_boxes[i]).owner = nil) {
					do write_cycle("No owner");
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
								do write_cycle("Better_box!!! Suppress target");
								targeted_box.owner <- nil;
								targeted_box <- reachable_boxes[i];
								targeted_box.owner <- self;
							}
						}
					// I carry a box
					} else {
						int my_target_box_crit <- compute_anticipated_criticality(carried_box);
						do write_cycle("mycarriedboxcrit:" + my_target_box_crit);
						// I check if this is a better box
						if (ant_reach_crit < my_target_box_crit) {
							do write_cycle("Better_box!!! Suppress carried");
							carried_box.owner <- nil;
							carried_box <- nil;
							is_carrying_box <- false;
							targeted_box <- reachable_boxes[i];
							targeted_box.owner <- self;
						}
					}	
				}
				i <- i + 1;
				if ((reachable_boxes count true) = 1) {
					end_reachable_boxes <- true;
				} else {
					end_reachable_boxes <- ((i + 1) = (reachable_boxes count true));
				}
			}
			
			//do write_cycle('serach box end:'+cell_boxes_buffer);
		}
		
		reflex go_to_target_box when: targeted_box != nil and is_auth_to_move and !is_dead{
			waiting_turn_nb <-0;
			path path_followed <- self goto [target::targeted_box, on::cell, return_path:: true, speed::robot_speed];
			myCell <- cell({location.x, location.y});
		}
		
		
		
		reflex take_box when: targeted_box != nil and (targeted_box.myCell = myCell) and !is_dead{
			//do write_cycle("mybox:" + targeted_box);
			carried_box <- targeted_box;
			targeted_box <- nil;
			carried_box.owner <- self;
			is_carrying_box <- true;
		}
		
		reflex carry_box_to_nest when: carried_box != nil and !is_waiting_to_give_box and is_auth_to_move and !is_dead{
			waiting_turn_nb <-0;
			path path_followed <- self goto [target::nests_locations[carried_box.color], on::cell, return_path:: true, speed::robot_speed];
			myCell <- cell({location.x, location.y});
			carried_box.myCell <- myCell;
			carried_box.location <- location;
		}
		
		reflex drop_box_in_nest when: carried_box != nil and carried_box.myCell = nests_locations[carried_box.color] {
			// Recompense
			//do write_cycle("I drop my box: " + carried_box);
			battery <- battery + colors_reward_efficiency[carried_box.color];
			carried_box.this_is_the_end <- true;
			//do write_cycle("dropmybox:" + carried_box);
			carried_box <- nil;
			is_carrying_box <- false;
			heading <- shuffle([0, 90, 180, 270])[0];
			do redefine_direction(orientation, heading);
			
		}
		
		reflex request_criticality when: is_need_a_box and carried_box = nil and targeted_box = nil and !is_request_criticality_last_cycle and !is_dead{
			
			list<robot_without_buffer> receivers <- robot_without_buffer where !each.is_dead inside myCell.neighbours_at_robot_speech;
			if (receivers != nil) {

				loop receiver over: receivers {
					if (receiver.is_carrying_box) {
						is_request_criticality_last_cycle <- true;
						do write_cycle('I request box to:' + receiver);
						string anticipated_criticality <- compute_anticipated_criticality(receiver.carried_box);
						do write_cycle(anticipated_criticality);
						do send with: [ receivers :: receiver, protocol :: 'fipa-request', performative :: 'request', content :: [criticality_string,anticipated_criticality, self.criticality] ];
					}
				}
			}
		}
		
		reflex read_requests when: !is_dead and !empty(requests){
			loop request over: requests {
				switch request.content[0] {
					match criticality_string {
						
						//do inform with: [ message :: request, content :: [criticality_string + ':' + criticality] ];
						// calcul la criticité anticipée et décide si oui ou non il laisse sa boite
						if (carried_box != nil) {
							int my_ant_criticality <- compute_anticipated_criticality(carried_box);
							int sender_ant_criticality <- int(request.content[1]);
							if (my_ant_criticality > sender_ant_criticality) {
								do agree with: [ message :: request, content :: [give_my_box_string] ];
								do inform with: [ message :: request, content :: [give_my_box_string, my_ant_criticality] ];
								do write_cycle('Propose my box(' + carried_box + ') ' + 'to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
							} else {
								do refuse with: [ message :: request, content :: ['Ko'] ];
								//do write_cycle('Dont propose my box to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
							}
						}
					}
					match demand_box_string {
						do agree with: [ message :: request, content :: [carried_box.location] ];
						do inform with: [ message :: request, content :: ['waiting'] ];
						robot_without_buffer robott <- (request.sender as robot_without_buffer);
						carried_box.owner <- robott;
						robott.targeted_box <- carried_box;
						//do write_cycle('Give my box to ' + request.sender + ' ' + carried_box);
						//do write_cycle("dropmybox:" + carried_box);
						carried_box <- nil;
						is_carrying_box <- false;
					}
				}
			}
		}
		
		reflex read_agrees when: !empty(agrees) and !is_dead{
			
			loop agree over: agrees {
				if (agree.content contains give_my_box_string) {
					is_request_criticality_last_cycle <- false;
					do write_cycle('Agent ' + agree.sender + ' says Ok for giving');
				} else {
					location <- (agree.content as point);
					////do write_cycle('Agent ' + agree.sender + ' gives me his box at ' + location);
				}
			}
		}
		
		reflex read_refuses when: !empty(refuses) and !is_dead{
			////do write_cycle("read refuses");
			is_request_criticality_last_cycle <- false;
		}
		
		reflex read_inform when: !empty(informs) and !is_dead{
			int best_criticality <- max_criticality;
			robot_without_buffer giver <- nil;
			do write_cycle("Receive inform");
			loop inform over: informs {
				string content_i <- inform.content[0];
				
				if (content_i contains give_my_box_string) {
					//list<string> content_list <- content_i split_with ":";
					int ant_criticality_temp <- inform.content[1];
					string s_message <- 'Receive criticality of ' + inform.sender + "(" + ant_criticality_temp + ") ";
					string s_critic <- 'best(' + best_criticality + ')';  
					do write_cycle(s_message + s_critic);
					if (ant_criticality_temp < best_criticality) {
						best_criticality <- ant_criticality_temp;
						giver <- (inform.sender as robot_without_buffer);
					}
				}
			}
			
			if (giver != nil) {
				do send with: [ receivers :: giver, protocol :: 'fipa-request', performative :: 'request', content :: [demand_box_string] ];
				//do write_cycle('Request box of ' + giver);
			}
		}
		
		reflex die when: battery <= 0 and !is_dead{

			if carried_box != nil {
				carried_box.owner <- nil;
				//do write_cycle("dropmybox:" + carried_box);
				carried_box <- nil;
			}
			
			if targeted_box != nil {
				targeted_box.owner <- nil;
			    targeted_box <- nil;
			}
			is_dead <- true;
			color <- rgb("black");
			//do die ;
			//messages <- [];
			//requests <- [];
		}
		
		aspect base {
			draw circle(size) color: color;
			draw square(size) color: rgb("white");
		}
		
		aspect base_3D {
			draw sphere(1) color: color;
			draw cube(size) color: rgb("white");
		}

	}

}







