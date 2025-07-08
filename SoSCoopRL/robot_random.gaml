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
    
    // ROBOT RELATED
    int robot_vision_range <- 3;
    int max_battery <- 300;
	int min_battery <- 0;
	int initial_battery <- max_battery;
	int battery_consum <- 1;  // battery consumed per movement step
	
	// REWARD RELATED
	int reward <- int((2 * max_battery) / 3);
	int reduced_reward <- int(0.2 * reward);
	   
    init {
    	do spawn_nests();
    	//do spawn_boxes();
    	create box_ number:100;
    	create robot_without_buffer number:90;
    }
    
    action spawn_nests { 
    	// nests are equidistanct to each other		    
	    create nest with: [ color:rgb("red"), location:cell[15,15].location ];
	    create nest with: [ color:rgb("green"), location:cell[35,15].location ];
	    create nest with: [ color:rgb("blue"), location:cell[25,32].location ];
	    //do log(string(nests_locations));
    }
    
   
	
	reflex debugfunc {
		//write one_of(cell);
	}   
}



grid cell width:grid_width height:grid_height neighbors: 4 {
    // list<cell> neighbors_at_robot_vision2 <-  cell at_distance(robot_vision_range); // includes diagonal distance
    //list<cell> neighbors_at_robot_vision <- (self neighbors_at (robot_vision_range));
    
    list<cell> neighbors_at_robot_vision update: (self neighbors_at (robot_vision_range));
       
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
	robot_without_buffer owner <- nil;
	//cell myCell update: cell first_with (each overlaps self);
	// cell myCell <- cell(location);  // fastest
	// cell myCell <- cell closest_to self;
	cell myCell update: cell closest_to self;	
	
	reflex printLoc when: DEBUG=true {
		write('Box:' + string(color) + 'CellIndex: '+ myCell+ string(myCell.grid_x) + myCell.grid_y + string(location));
	}
	
	
	init {
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


species robot_without_buffer skills: [moving] {
    // ===== ATTRIBUTES =====
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
	
	// ===== UPDATE BATTERY =====
	reflex update_battery when: battery > 0 {
		bool moved_step <- (location != previous_location);
		if (moved_step) {
			battery <- battery - battery_consum;
            // write name + " moved, battery now: " + battery;
            write "battery: " + battery + ' for ' + name;
			
		}
		previous_location <- location;
	}
	
	
	
    // ===== SEARCH & TARGET =====
    reflex search_for_boxes when: empty(reachable_boxes) and targeted_box=nil and battery > 0 {
        do wander amplitude: 90.0;        
    }
    
    reflex target_best_box when: !empty(reachable_boxes) and targeted_box=nil and carried_box=nil and battery > 0 {
        // First, try to find a box of my color
        list<box_> my_color_boxes <- reachable_boxes where (each.color = color);
        
        if !(empty(my_color_boxes)) {
        	targeted_box <- my_color_boxes closest_to self;
        } else {    	
	        targeted_box <- reachable_boxes closest_to self;
        }
        
        targeted_box.owner <- self;
    }

    // ===== MOVEMENT & PICKUP =====
    reflex go_to_target_box when: targeted_box != nil and battery > 0 {
        path path_followed <- goto(target: targeted_box.location, on: cell, return_path: true, speed: float(robot_speed));
        //write 'going towards box';    
        
        if targeted_box.myCell=myCell {
            //write 'reached at the target box';
        }
    }
    
    reflex take_box when: targeted_box!=nil and targeted_box.myCell=myCell and targeted_box.owner=self and battery > 0 {
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
	        if (carried_box.color=color) {
	        	battery <- battery + reward;  // Same color: full reward
	        } else {
	        	battery <- battery + reduced_reward;  // other color: 20% reward
	        }
	        
	        ask carried_box { do die; }
	        
	        carried_box <- nil;
	        target_nest <- nil;
	    } else {
	        do goto target: target_nest on: cell return_path: true speed: float(robot_speed);
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
    


} // end of robot_without_buffer species



experiment myExperiment type:gui {
	output {
		display myDisplay type:opengl {
			grid cell border: #black;
			species cell aspect:default;
			species nest aspect:default;
			species box_ aspect:default;
			species robot_without_buffer aspect:default;
		}
	}
}



