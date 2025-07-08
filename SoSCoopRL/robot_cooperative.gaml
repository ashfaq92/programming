/**
* Name: robot_random 
* Author: ashfa
* Tags: CoCaRo
*/


model robot_random

global {
	float seed <- 0.0 parameter:true;
	
	// DEBUGGING
	bool DEBUG <- false;
	
	// GENERAL
    list<rgb> colors <- [rgb("red"), rgb("green"), rgb("blue")];
	
	
	// GRID RELATED
	int grid_width <- 50;
	int grid_height <- 50;
	// NEST RELATED
	
	// BOX RELATED
    int box_creation_interval <- 3;  // Create a box every 3 cycles
    int box_creation_counter <- 0;    // Counter to track creation timing
	int max_boxes_buffer <- 2;
    
    // ROBOT RELATED
    int robot_vision_range <- 3;
    int robot_speech_range <- 3;
    int max_battery <- 300;
	int min_battery <- 0;
	int initial_battery <- max_battery;
	int battery_consum <- 1;  // battery consumed per movement step
	
	// REWARD RELATED
	int reward <- int((2 * max_battery) / 3);
	int reduced_reward <- int(0.2 * reward);
	
	// CRITICALITY RELATED
	int min_criticality <- min_battery;
	int max_criticality <- max_battery;
	int need_box_threshold <- int(max_criticality / 2);
	string criticality_string <- "criticality";
	string demand_box_string <- "GiveMeYourBox";
	string give_my_box_string <- "GiveMyBoxToYou";
	   
    init {
    	do spawn_nests();
    	//do spawn_boxes();
    	create box_ number:100;
    	create robot_cooperative number: 90;
    }
    
    action spawn_nests { 
    	// nests are equidistanct to each other		    
	    create nest with: [ color:rgb("red"), location:cell[15,15].location ];
	    create nest with: [ color:rgb("green"), location:cell[35,15].location ];
	    create nest with: [ color:rgb("blue"), location:cell[25,32].location ];
	    //do log(string(nests_locations));
    }
    
    

	reflex spawn_boxes_at_interval {
	    box_creation_counter <- box_creation_counter + 1;
	    if (box_creation_counter >= box_creation_interval) {
	        create box_ number: 1;  // This calls the species init which calls create_box
	        box_creation_counter <- 0;
	        write 'box spawned at ' + cycle;
	    }
	} 
	
	reflex debugfunc {
		//write one_of(cell);
	}   
}



grid cell width:grid_width height:grid_height neighbors: 4 {    
    list<cell> neighbors_at_robot_vision update: (self neighbors_at (robot_vision_range));
    list<cell> neighbors_at_robot_speech update: (self neighbors_at (robot_speech_range));
       
  	aspect default {
	    draw string(string(index)) color: #black anchor: #center font: font(3); 
  	}
}

species nest {
	int deposited_boxes <- 0;
	rgb color;
	//cell myCell update: cell first_with (each overlaps self);
	//cell myCell <- cell(location);  // fastest
	cell myCell <- cell closest_to self;	
	
	reflex printLoc when: DEBUG=true {
		write('Nest: ' + string(color) + 'CellIndex: '+ myCell+ string(myCell.grid_x) + myCell.grid_y + string(location));
	}
	
	aspect default {        
		draw circle(1) color: #white border: color;
		draw string("Nest") color: color size:1;			
    }
    
    
    aspect icon {        
    	image_file nest_shape <- file('../images/bin.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw nest_shape size: {cell_width, cell_height}; 
    }
    
    
}


species box_ {
	rgb color;
	robot_cooperative owner <- nil;
	//cell myCell update: cell first_with (each overlaps self);
	// cell myCell <- cell(location);  // fastest
	// cell myCell <- cell closest_to self;
	cell myCell update: cell closest_to self;	
	
	reflex printLoc when: DEBUG=true {
		write('Box:' + string(color) + 'CellIndex: '+ myCell+ string(myCell.grid_x) + myCell.grid_y + string(location));
	}
	
	action create_box {
		// get random color
    	color <- any(colors);
    	// get random location on grid
    	loop while: true {
    		cell randCell <- any(cell);
    		//bool isEmpty <- length(agents_on(randCell)) = 0;
    		bool isEmpty <- length(agents_inside(randCell)) = 0;
    		if (isEmpty) {
    			//myCell <- randCell;
    			location <- randCell.location;
    			break;
    		}    		
    	}
	}
	
	init {
    	do create_box();
    }
   
    
    
	
	aspect default {        
		draw square(1) color:color; 
		draw string("Box") color: color size:0.5;			
    }
    
    aspect icon {
    	image_file box_shape <- file('../images/box.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw box_shape  size: {cell_width, cell_height};
    }
}


species robot_cooperative skills: [moving, fipa] {
    // ===== ATTRIBUTES =====
	bool is_need_a_box update: need_box_threshold < criticality;
    int criticality <- 0 update: max_criticality - battery max: max_criticality;

	
	int battery <- initial_battery min: min_battery max: max_battery; 
	point previous_location <- location;  // for movement identification to update battery
    
    string previous_state <- "";
    //string current_state <- "searching";  // "searching", "targeting", "carrying", "depositing"
    nest target_nest <- nil;
    box_ carried_box <- nil;
    int robot_speed <- 0;
    box_ targeted_box <- nil;
    list<box_> reachable_boxes update: (box_ where (myCell.neighbors_at_robot_vision contains each.myCell)) where (each.owner = nil);
    rgb color;
    cell myCell update: cell closest_to self;

    // ===== INITIALIZATION =====
	init {
	    speed <- 1.0;
	    // get random color
	    color <- any(colors);
	    
	    // get random location on grid
	    loop while: true {
	        cell randCell <- any(cell);
	        //bool isEmpty <- length(agents_on(randCell)) = 0;
	        bool isEmpty <- length(agents_inside(randCell)) = 0;
	        if (isEmpty) {
	            //myCell <- randCell;
	            location <- randCell.location;
	            break;
	        }            
	    }
	    
	    cell cell0 <- cell[0,0];
	    cell cell1 <- cell[1,0];
	    robot_speed <- int(cell1.location.x - cell0.location.x);
	}

    
	// ===== DIE =====
	reflex die when: battery <= 0 {
	    write name + " battery depleted!";
	    
	    // Release carried box (don't destroy it)
	    if (carried_box != nil) {
	        carried_box.owner <- nil;  // Other robots can pick it up
	        carried_box <- nil;
	    }
	    
	    // Release targeted box claim
	    if (targeted_box != nil) {
	        targeted_box.owner <- nil;  // Other robots can target it
	        targeted_box <- nil;
	    }    
	    color <- rgb("black");
	}
	
	int colors_reward_efficiency(rgb box_color) {
		if (box_color=color) {
        	return reward;  // Same color: full reward
        } else {
        	return reduced_reward;  // other color: 20% reward
        }
	}
	
	
	
	// ===== UPDATE BATTERY =====
	reflex update_battery when: battery > 0 {
		bool moved_step <- (location != previous_location);
		if (moved_step) {
			battery <- battery - battery_consum;			
		}
		previous_location <- location;
	}
	
	
	
    // ===== MOVE & SEARCH =====
    reflex basic_move when: targeted_box=nil and carried_box=nil and battery > 0 {
        do wander amplitude: 90.0;        
    }
    
    reflex search_box when: !empty(reachable_boxes) and battery > 0 {
    	loop bx over: reachable_boxes {
    		// bool message sent <- false
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

    // ===== MOVEMENT & PICKUP =====
    reflex go_to_target_box when: targeted_box != nil and battery > 0 {
        path path_followed <- goto(target: targeted_box.location, on: cell, return_path: true, speed: float(robot_speed));
        //write 'going towards box';    
        
        if targeted_box.myCell=myCell {
            //write 'reached at the target box';
        }
    }
    
    reflex take_box when: targeted_box!=nil and targeted_box.myCell=myCell and battery > 0 {
        carried_box <- targeted_box;
        targeted_box <- nil;
    }

    // ===== CARRYING & DEPOSIT =====
    reflex carry_box_to_nest when: carried_box != nil and battery > 0 {
        target_nest <- nest first_with (each.color = carried_box.color);
        do goto target: target_nest on: cell return_path: true speed: float(robot_speed);
    }
    
    reflex update_carried_box_position when: carried_box != nil and battery > 0 {
        carried_box.myCell <- myCell;
        carried_box.location <- location;
    }
    
    reflex drop_box_in_nest when: carried_box != nil and target_nest != nil and battery > 0 {
	    if (myCell = target_nest.myCell) {
	        target_nest.deposited_boxes <- target_nest.deposited_boxes + 1;
	        // write('box deposited at ' + string(target_nest.color) + ' nest! Total: ' + string(target_nest.deposited_boxes));
	        
	        // reward system	        
	        battery <- battery + colors_reward_efficiency(carried_box.color);
	        
	        ask carried_box { do die; }
	        
	        carried_box <- nil;
	        target_nest <- nil;
	    } else {
	        do goto target: target_nest on: cell return_path: true speed: float(robot_speed);
	    }
	}
	
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
			return max_battery;
		} else {
			return max_criticality - anticipated_battery;
		}
	}
	
	
	// ===== COOPERATION WITH OTHER ROBOTS =====
	reflex request_criticality when: is_need_a_box and carried_box=nil and targeted_box=nil and battery>0 {		
		list<robot_cooperative> receivers <- (robot_cooperative where (myCell.neighbors_at_robot_speech contains each.myCell)) where (each.battery > 0);
				
		loop receiver over: receivers {
			if receiver.carried_box != nil {
				write 'i request box to:' + receiver;
				int anticipated_criticality <- compute_anticipated_criticality(receiver.carried_box);
				do start_conversation to: [receiver] protocol:'fipa-request' performative: 'request' contents: [criticality_string, anticipated_criticality, self.criticality];
			}
		}
	}
	
	reflex read_request when: !empty(requests) and battery > 0 {
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
			
			if (request_type=demand_box_string) {
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
	
	
	// ===== ASPECTS =====
    aspect default {        
        draw triangle(1) color: color;
        draw string("Robot") color: color size: 0.5;            
    }
    
    aspect icon {
    	image_file robot_shape <- file('../images/robot.png');
		float cell_width <- world.shape.width / 50;
      	float cell_height <- world.shape.height / 50;
		draw robot_shape size: {cell_width, cell_height} rotate: my heading + 1 wireframe: false; 
    }

    // ===== DEBUG =====
    reflex printLoc when: DEBUG {
        write('Robot:' + string(color) + 'CellIndex: '+ myCell+ string(myCell.grid_x) + myCell.grid_y + string(location));
    }
    


} // end of robot_cooperative species



experiment myExperiment type:gui {
	output {
		display myDisplay type:opengl {
			grid cell border: #black;
			species cell aspect:default;
			species nest aspect:default;
			species box_ aspect:default;
			species robot_cooperative aspect:default;
		}
	}
}



