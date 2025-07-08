/**
* Name: robotwithoutbufferashfaq
* Based on the internal empty template. 
* Author: ashfa
* Tags: 
*/


model robotwithoutbufferashfaq

global {    
	float seed <- 0.0 parameter:true;
	
	// GRID RELATED
    int grid_width <- 50;
    int grid_height <- 50;

    // Direction constants (integers for map keys)
    int FORWARD <- 0;
    int BACK <- 1;
    int LEFT <- 2;
    int RIGHT <- 3;
    
    // Cardinal constants (strings for map values)
    string NORTH <- "NORTH";
    string SOUTH <- "SOUTH";
    string EAST <- "EAST";
    string WEST <- "WEST";
    

	// ROBOT RELATED
	int max_battery <- 300;
	int reward <- int((2 * max_battery) / 3);
	float counter_efficiency <- 0.2;
	int min_battery <- 0;
	int max_criticality <- max_battery;
	int min_criticality <- min_battery;
	int initial_battery <- max_battery;
	int battery_consum <- 1;
	int need_box_threshold <- int(max_criticality / 2);
	int battery_state_nb <- 3;
	string criticality_string <- "criticality";
	string demand_box_string <- "GiveMeYourBox";
	string give_my_box_string <- "GiveMyBoxToYou";
	int robot_vision_range <- 3;

	// BOX RELATED
    int box_creation_interval <- 3;  // Create a box every 3 cycles
    int box_creation_counter <- 0;    // Counter to track creation timing
	int max_boxes_buffer <- 2;

	// NEST RELATED
    map<rgb, cell> nests_locations <- map([]); // Initialize as empty map


	init {
		do spawn_nests();
	    do spawn_boxes();
	    do spawn_robots();
	}
	
	action spawn_nests {
		// Define equidistant location points for nests
	    list<cell> nest_cells <- shuffle([cell[15,15], cell[35,15], cell[25,32]]);
    	// possible colors
    	list<rgb> possible_colors <- shuffle([rgb("red"), rgb("green"), rgb("blue")]);
    	//creat 3 nests at specific locations
    	loop i from: 0 to: 2 {
    		
    		list<cell> empty_cells <- cell where (
	        	empty(nest inside each) and 
	        	empty(box_ inside each) and
                empty(robot_without_buffer inside each)
	        ); 
	        
	        if !empty(empty_cells) {
	    		create nest with: [
	    			location:nest_cells[i].location,
	    			color:possible_colors[i]	
	    		];
    		}
    	}
    	
    	
		loop n over:nest {
			nests_locations[n.color] <- n.myCell;
		}	
	}
	
	action spawn_boxes {
		// Create 10 initial boxes with random colors at random empty locations
	    list<rgb> box_colors <- [rgb("red"), rgb("green"), rgb("blue")];
	    
	    loop i from: 0 to: 9 {
	        // Find empty cells (no nests, no other boxes)
	        list<cell> empty_cells <- cell where (
	        	empty(nest inside each) and 
	        	empty(box_ inside each) and
                empty(robot_without_buffer inside each)
	        );
	        
	        if !empty(empty_cells) {
	            create box_ with: [
	                location :: one_of(empty_cells).location,
	                color :: one_of(box_colors)
	            ];
	        }
	    }
		
	}
	
	action spawn_robots {
		// Create 90 robots (30 of each color)
	    list<rgb> robot_colors <- [rgb("red"), rgb("green"), rgb("blue")];
	    
	    loop color_rgb over: robot_colors {
	        loop i from: 0 to: 29 {  // 30 robots per color
	            // Find empty cells (no nests, no boxes, no other robots)
	            list<cell> empty_cells <- cell where (
	                empty(nest inside each) and 
	                empty(box_ inside each) and 
	                empty(robot_without_buffer inside each)
	            );
	            
	            if !empty(empty_cells) {
	                create robot_without_buffer with: [
	                    location: one_of(empty_cells).location,
	                    color: color_rgb
	                ];
	            }
	        }
	    }
		
	}
	
	reflex end_cycle {
		box_creation_counter <- box_creation_counter + 1;
		if (box_creation_counter >= box_creation_interval) {
			do create_new_box();
			box_creation_counter <- 0;
		}
	}
	
	reflex initialize_nest_locations when: empty(nests_locations) {
        loop n over: nest {
            nests_locations[n.color] <- n.myCell;
        }
        write "Nests locations initialized: " + nests_locations;
    }
	
	action create_new_box {
		// Find empty cells (no nests, no other boxes, no robots)
        list<cell> empty_cells <- cell where (
            empty(nest inside each) and 
            empty(box_ inside each) and
            empty(robot_without_buffer inside each)
        );
        
        if !empty(empty_cells) {
            create box_ with: [
                location :: one_of(empty_cells).location,
                color :: one_of([rgb("red"), rgb("green"), rgb("blue")])
            ];
            write "Created new box at cycle " + cycle;
        }
	}
	
	
}

grid cell width:grid_width height:grid_height neighbors: 4 {
	// docs: https://gama-platform.org/wiki/GridSpecies
	// note: columns=grid_x, rows=gird_y
	//
	
    // Computed attribute that returns cells within robot vision	
    	

    // geometry neighbors_at_robot_vision update: union((self neighbors_at robot_vision_range) collect each.geometry);
    
    // robots are allowed to move through each other, 
    // so no deadlocks due to overcrowding could occur.

    // As robots move one square per simulation step with no diagonal movement, we dont need diagonal neighbors

    // geometry neighbors_at_robot_vision update: union(union([self], self neighbors_at robot_vision_range) collect each.shape); // VERIFY!
    list<cell> neighbors_at_robot_vision <- (self neighbors_at (robot_vision_range));
    
}

species box_ {
	rgb color;	
	robot_without_buffer owner <- nil;
	// Boolean flag (indicates if box reached nest)
    bool this_is_the_end <- false;
	
	// reference to the cell containing this box
	cell myCell update: cell closest_to (self.location);
	//cell myCell update: cell first_with (each overlaps self);
	
	aspect default {
        image_file box_shape <- file('../images/box.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw box_shape  size: {cell_width, cell_height};
        
    }
}

species nest {
	rgb color;
	//cell myCell update: cell first_with (each overlaps self);	
	cell myCell <- nil;
	
	init {
		myCell <- cell first_with (each overlaps self);
	} 
	
	
	
	aspect default {        
    	image_file nest_shape <- file('../images/bin.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw nest_shape size: {cell_width, cell_height}; 
    }
}



species robot_without_buffer skills:[moving, fipa] {
	float size <- 1.0;
	//update: battery - battery_consum min: min_battery max: max_battery ;
	int battery <- initial_battery min: min_battery max: max_battery;
	rgb color;
	int robot_speed <- 0;
	
	//const efficiency type: map <- ['red'::rnd (10) / 10 , 'green'::rnd (10) / 10, 'blue'::rnd (10) / 10];
	cell myCell;
	list<box_> reachable_boxes update: box_ inside (myCell.neighbors_at_robot_vision);
	
	box_ targeted_box <- nil;
	box_ carried_box <- nil;
	
	bool is_carrying_box <- false;
	bool is_empty_battery <- false;
	bool is_waiting_to_give_box <- false;
	bool is_need_a_box update: need_box_threshold < criticality;
	bool is_request_criticality_last_cycle <- false;
	bool is_diry_orientation <- false;
	bool is_auth_to_move update: waiting_turn_nb >= battery_state;
	bool is_dead <- false;
	// Baterry state:
	// 0: High ()
	// 1: Medium ()
	// 2: Low ()
	int battery_state <- (battery_state_nb - 1) min:0 max: (battery_state_nb - 1) update: int(battery_state_nb - battery_state_nb * (battery / max_battery));
	int waiting_turn_nb <- 0;
	int criticality <- 0 update: max_criticality - battery max: max_criticality;
	map<int, string> orientation <- nil;
	map<rgb, int> colors_reward_efficiency <- nil;
	
	
	action write_cycle(string message_) {
		string sim_cycle <- 'cycle ' + cycle + ', ' + self + ': ';
		write sim_cycle + message_;
	}
	
	action redefine_direction(float heading_temp) {
		switch heading_temp {
			// robot oriented to the east
			match 0.0 { 
				orientation[FORWARD] <-  EAST;
				orientation[BACK] <-  WEST;
				orientation[LEFT] <-  NORTH;
				orientation[RIGHT] <- SOUTH;
			}
			// robot oriented to the south
			match 90.0 {
				orientation[FORWARD] <-  SOUTH;
				orientation[BACK] <-  NORTH;
				orientation[LEFT] <-  EAST;
				orientation[RIGHT] <- WEST;
			}
			// robot oriented to the west
			match 180.0 {
				orientation[FORWARD] <-  WEST;
				orientation[BACK] <-  EAST;
				orientation[LEFT] <-  SOUTH;
				orientation[RIGHT] <- NORTH;
			}
			// robot oriented to the north
			match 270.0 {
				orientation[FORWARD] <-  NORTH;
				orientation[BACK] <-  SOUTH;
				orientation[LEFT] <-  WEST;
				orientation[RIGHT] <-  EAST;
			}
		}
	}
	
	int monte_carlo_draw {
		int direction <- rnd(100);
		
		// go forward
		if direction <= 90 {
			return FORWARD;
		// go left
		} else if direction <= 95 {
			return LEFT;
		// go right
		} else if direction <= 100 {
			return RIGHT;
		// go back
		} else {
			return BACK;
		}
	}
	
	action set_reward_efficiency(rgb colorArg) {
		int not_so_good_reward <- int (reward * counter_efficiency);
		
		switch colorArg {
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
	
	
	int compute_anticipated_criticality(box_ box_to_take) {
		// float dist_box_to_me <- abs(self.myCell.y - box_to_take.myCell.location.y) + abs(self.location.x - box_to_take.location.x);
		
	 	if (box_to_take = nil) { return max_criticality; }  // Guard clause
	 	if (empty(nests_locations)) { return max_criticality; }  // Guard clause	
	 	
	 	
	 	 // *** NEW GUARD CLAUSE ***
	    if (box_to_take.myCell = nil) {
	        do write_cycle("Warning: box_to_take.myCell is nil for box: " + box_to_take);
	        return max_criticality; // Cannot compute if box has no valid cell
	    }
		
		int dist_box_to_me <- abs(box_to_take.myCell.grid_y - self.myCell.grid_y) + abs(box_to_take.myCell.grid_x - self.myCell.grid_x);
		
		cell nest_cell <- nests_locations[box_to_take.color];
		int dist_robot_to_nest <- abs(nest_cell.grid_y - box_to_take.myCell.grid_y) + abs(nest_cell.grid_x - box_to_take.myCell.grid_x);
		
        
 
        
        int anticipated_battery_before_reward <- int(battery - (dist_box_to_me + dist_robot_to_nest) * battery_consum);
        
		if anticipated_battery_before_reward < 0 {
			anticipated_battery_before_reward <- 0;
		}
		
		int anticipated_battery <- anticipated_battery_before_reward + colors_reward_efficiency[box_to_take.color];
		
		if (anticipated_battery < min_battery) {
			return max_criticality;
		} else if (anticipated_battery > max_battery) {
			return min_criticality;
		} else {
			return max_criticality - anticipated_battery;
		}
		
	}
	
	init {
		heading <- any([0.0, 90.0, 180.0, 270.0]);
		
		do redefine_direction(heading);
		
		do set_reward_efficiency(color);
		
		myCell <- one_of(cell);
		
		list<robot_without_buffer> robots <- robot_without_buffer inside myCell;
		location <- myCell.location;
		loop while: (! empty(robots)) {
			myCell <- one_of (cell);
			robots <- robot_without_buffer inside myCell;
			location <- myCell.location;
		}
		cell cell0 <- cell[0,0];
		cell cell1 <- cell[1,0];
		robot_speed <- int(cell1.location.x - cell0.location.x);
	}
	
	reflex update_battery_state when: !is_dead and battery > 0 {
		if (is_auth_to_move) {
			battery <- battery - battery_consum;
		}
	}
	
	reflex info when: !is_dead {
//		write "battery_state:" + battery_state + ";" + battery;
//		write "waiting_turn_nb:" + waiting_turn_nb;
//		write "is_auth_to_move:" + is_auth_to_move;
	}
	
	reflex count_waiting_turn when: !is_auth_to_move and !is_dead {
		waiting_turn_nb <- waiting_turn_nb + 1;
	}
	
	reflex basic_move when: targeted_box=nil and carried_box=nil and is_auth_to_move and !is_dead {
		waiting_turn_nb <- 0;
		// Monte carlo draw
		do redefine_direction(heading);
		
		cell temp_cell <- nil;
		loop while:temp_cell=nil {
			string goto <- orientation[monte_carlo_draw()];
			
			
			switch goto {
				// robot oriented to the north
				match NORTH {
					temp_cell <- cell[myCell.grid_x, myCell.grid_y - 1];
				}
				// robot oriented to the south
				match SOUTH {
					temp_cell <- cell[myCell.grid_x, myCell.grid_y + 1];
				}
				// robot oriented to west
				match WEST {
					temp_cell <- cell[myCell.grid_x - 1, myCell.grid_y];
				}
				// robot oriented to east
				match EAST {
					temp_cell <- cell[myCell.grid_x + 1, myCell.grid_y];
				}
			}
		}
		myCell <- temp_cell;
		location <- myCell.location;
	}
	
	reflex search_box when: !empty(reachable_boxes) and !is_dead {
		bool end_reachable_boxes <- false;
		int i <- 0;
		loop while: !end_reachable_boxes {
			bool message_sent <- false;
			int box_efficiency <- colors_reward_efficiency[(reachable_boxes[i]).color];
			int ant_reach_crit <- compute_anticipated_criticality((reachable_boxes[i]));
			// No Owner
			if ((reachable_boxes[i]).owner=nil) {
				do write_cycle('no owner');
				// I dont carry a box, too
				if (carried_box=nil) {
					// i dont target a box
					if (targeted_box=nil) {
						targeted_box <- reachable_boxes[i];
						targeted_box.owner <- self;
					// i target a box
					} else {
						int my_target_box_crit <- compute_anticipated_criticality(targeted_box);
						// i check if this is a best box
						if (ant_reach_crit < my_target_box_crit) {
							do write_cycle('Better box!!! Suppress target');
							targeted_box.owner <- nil;
							targeted_box <- reachable_boxes[i];
							targeted_box.owner <- self;
						}
					}
				// I carry a box
				} else {
					int my_carried_box_crit <- compute_anticipated_criticality(carried_box);
					do write_cycle("mycarriedboxcrit " + my_carried_box_crit);
					// i check if this is the best box
					if (ant_reach_crit < my_carried_box_crit) {
						do write_cycle("Better box!!! Suppress carried");
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
				end_reachable_boxes <- ( (i+1) >= (reachable_boxes count true) );
			}
		}
	}
	
	reflex go_to_target_box when: targeted_box!=nil and is_auth_to_move and !is_dead {
		waiting_turn_nb <- 0;
	    path path_followed <- goto(target: targeted_box.location, on: cell, return_path: true, speed: float(robot_speed));
		//path path_followed <- goto([target:targeted_box, on:cell, return_path:true, speed:robot_speed]);  // BEWARE of 'location'
		myCell <- cell({location.x, location.y});		
		
		
		// another pathfinding syntax:
        //path path_followed <- path_to(target: targeted_box, on: cell);
        //do follow(target: path_followed, speed: robot_speed);
        //myCell <- cell covering (location); // Update current cell
	}
	
	reflex take_box when:targeted_box!=nil and (targeted_box.myCell=myCell) and !is_dead {
		do write_cycle('picked up the box!');
		carried_box <- targeted_box;
		targeted_box <- nil;
		carried_box.owner <- self;
		is_carrying_box <- true;
	}
	
	reflex carry_box_to_nest when:carried_box!=nil and !is_waiting_to_give_box and is_auth_to_move and !is_dead {
		waiting_turn_nb <- 0;
		do write_cycle('carrying box to nest!');
		//do goto target: nests_locations[carried_box.color] on:cell return_path:true speed:float(robot_speed);
		do goto target: nests_locations[carried_box.color] on:cell return_path:true speed:float(robot_speed);
		myCell <- cell({location.x, location.y});
		carried_box.myCell <- myCell;
		carried_box.location <- location;
	}
	
//	reflex drop_box_in_nest when: carried_box!=nil and carried_box.myCell=nests_locations[carried_box.color] and !is_dead {  //check cell comparison with location comparison
//		do write_cycle('box deposited to nest!');
//		// Recompense
//		battery <- battery + colors_reward_efficiency[carried_box.color];
//		carried_box.this_is_the_end <- true;
//		carried_box <- nil;
//		is_carrying_box <- false;
//		heading <- shuffle([0.0, 90.0, 180.0, 270.0])[0];
//		do redefine_direction(heading);
//	}
	
//	reflex drop_box_in_nest when: carried_box != nil and !is_dead and nests_locations contains carried_box.color {
//	    cell nest_cell <- nests_locations[carried_box.color];
//	    
//	    if (carried_box.myCell.grid_x = nest_cell.grid_x and 
//	        carried_box.myCell.grid_y = nest_cell.grid_y) {
//	        do write_cycle('box deposited to nest!');
//	        battery <- battery + colors_reward_efficiency[carried_box.color];
//	        carried_box.this_is_the_end <- true;
//	        carried_box <- nil;
//	        is_carrying_box <- false;
//	        heading <- shuffle([0.0, 90.0, 180.0, 270.0])[0];
//	        do redefine_direction(heading);
//	    }
//	}
	
	reflex drop_box_in_nest when: carried_box != nil and myCell overlaps nests_locations[carried_box.color] and !is_dead {
	    do write_cycle('box deposited to nest!');
	    // Recompense
	    battery <- battery + colors_reward_efficiency[carried_box.color];
	    carried_box.this_is_the_end <- true;
	    carried_box <- nil;
	    is_carrying_box <- false;
	    heading <- shuffle([0.0, 90.0, 180.0, 270.0])[0];
	    do redefine_direction(heading);
	}
	
	reflex request_criticality when:is_need_a_box and carried_box=nil and targeted_box=nil and !is_request_criticality_last_cycle and !is_dead {
		list<robot_without_buffer> receivers <- robot_without_buffer where !each.is_dead inside myCell.neighbors_at_robot_vision;
		if (receivers!=nil) {
			loop receiver over: receivers {
				if (receiver.is_carrying_box) {
					is_request_criticality_last_cycle <- true;
					do write_cycle('I request box to: ' + receiver);
					int anticipated_criticality <- compute_anticipated_criticality(receiver.carried_box);
					do write_cycle(string(anticipated_criticality));
					do start_conversation to:[receiver] protocol:'fipa-request' performative:'request' contents:[criticality_string, anticipated_criticality, self.criticality];
				}
			}
		}
	}
	
	
	reflex read_requests when: !empty(requests) and !is_dead{
		loop request over: requests {
			switch request.contents[0] {
				match criticality_string {
					// calculates the anticipated criticality and decides whether or not to give up its box.
					if (carried_box != nil) {
						int my_ant_criticality <- compute_anticipated_criticality(carried_box);
						int sender_ant_criticality <- int(request.contents[1]);
						if (my_ant_criticality > sender_ant_criticality) {
							do agree with: [ message:request, contents: [give_my_box_string] ];
							do inform with: [message:request, contents:[give_my_box_string, my_ant_criticality]];
							do write_cycle('Propose my box(' + carried_box + ') ' + 'to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');
						} else {
							do refuse with: [message:request, contents:['Ko']];
							do write_cycle('Did not propose my box to Agent ' + request.sender + '(' + sender_ant_criticality + '), me(' + my_ant_criticality + ')');					
						}
					}
				}
				match demand_box_string {
					do agree with: [ message:request, contents:[carried_box.location] ];
					do inform with: [ message:request, contents:['waiting'] ];
					robot_without_buffer robott <- (request.sender as robot_without_buffer);
					carried_box.owner <- robott;
					robott.targeted_box <- carried_box;
					carried_box <- nil;
					is_carrying_box <- false;
				}
			}
		}
	}
	
	reflex read_agrees when:!empty(agrees) and !is_dead {
		loop agree over: agrees {
			if (agree.contents contains give_my_box_string) {
				is_request_criticality_last_cycle <- false;
				do write_cycle('Agent ' + agree.sender + 'says ok for giving.');					
			} else {
				location <- (agree.contents as point);
			}
		}
	}
	
	reflex read_refuses when: !empty(refuses) and !is_dead {
		is_request_criticality_last_cycle <- false;
	}
	
	reflex read_inform when: !empty(informs) and !is_dead {
		int best_criticality <- max_criticality;
		robot_without_buffer giver <- nil;
		do write_cycle('receive inform');
		
		loop inform over: informs {
			string content_i <- inform.contents[0];
			
			if (content_i contains give_my_box_string) {
				int ant_criticality_temp <- int(inform.contents[1]);
				string s_message <- 'Received criticality of ' + inform.sender + "(" + ant_criticality_temp + ")";
				string s_critic <- 'best(' + best_criticality + ')';
				do write_cycle(s_message + s_critic);
				if (ant_criticality_temp < best_criticality) {
					best_criticality <- ant_criticality_temp;
					giver <- (inform.sender as robot_without_buffer);
				}
			}
		}
		if (giver != nil) {
			//do send with: [ receivers::giver, protocol::'fipa-request', performative::'request', content::[demand_box_string] ];
			do start_conversation to:[giver] protocol:'fipa-request' performative:'request' contents:[demand_box_string];
		}	
	}
	
	reflex die when: battery <= 0 and !is_dead{
		write 'robot died (`_`)';
		if carried_box != nil {
			carried_box.owner <- nil;
			carried_box <- nil;
		}
		
		if targeted_box != nil {
			targeted_box.owner <- nil;
		    targeted_box <- nil;
		}
		is_dead <- true;
		color <- rgb("black");
	}
	
	
	
	aspect icon {
    	image_file robot_shape <- file('../images/robot.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw robot_shape size: {cell_width, cell_height} rotate: my heading + 1 wireframe: false; 
    }
	
	
	
}



experiment myExperiment type:gui {
	output {
		display myDisplay type:opengl {
			grid cell border: #black;
			species nest aspect:default;
			species box_ aspect:default;
			species robot_without_buffer aspect:icon;
		}
	}
}































