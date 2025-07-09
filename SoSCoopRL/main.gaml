model main
import "cell_.gaml"
import "nest.gaml"
import "box_.gaml"
import "robot_random.gaml"
import "robot_greedy.gaml"
import "robot_cooperative.gaml"

global {
    float seed <- 0.0 parameter:true;
    int box_creation_interval <- 3;  // Create a box every 3 cycles
    int box_creation_counter <- 0;    // Counter to track creation timing
    
    // Move the action here where all species are available
    action spawn_nests { 
        create nest with: [ color:rgb("red"), location:cell[15,15].location ];
        create nest with: [ color:rgb("green"), location:cell[35,15].location ];
        create nest with: [ color:rgb("blue"), location:cell[25,32].location ];
    }
       
    init {
        do spawn_nests();
        create box_ number:100;
        create robot_random number: 30;
        create robot_greedy number: 30;
        create robot_cooperative number: 30;
    }
    
    
	reflex spawn_boxes_at_interval {
	    box_creation_counter <- box_creation_counter + 1;
	    if (box_creation_counter >= box_creation_interval) {
	        create box_ number: 1;  // This calls the species init which calls create_box
	        box_creation_counter <- 0;
	        write 'box spawned at ' + cycle;
	    }
	} 
        
    reflex count_species {
	    int number_of_boxes <- length(box_);
	    
	    // Count all robot types individually
	    int number_of_robots <- length(robot_random) + length(robot_greedy) + length(robot_cooperative);
	    
	    // Or alternatively, count all agents with moving skill
	    // int number_of_robots <- length(agents of_species robot_random) + length(agents of_species robot_greedy) + length(agents of_species robot_cooperative);
	    
	    write('boxes: ' + string(number_of_boxes) );
	    write('robots: ' + string(number_of_robots) );
	    write('random robots: ' + string(length(robot_random)) );
	    write('greedy robots: ' + string(length(robot_greedy)) );
	    write('cooperative robots: ' + string(length(robot_cooperative)) );
	    
	    if number_of_boxes <= 0 or number_of_robots <= 0{
	        ask world {
	            do pause;
	        }    
	    }
	}
}

experiment myExperiment type:gui {
    output {
        display myDisplay type:opengl {
            grid cell border: #black;
            species cell aspect:default;
            species nest aspect:default;
            species box_ aspect:default;
            species robot_random aspect:default;
            species robot_greedy aspect:default;
            species robot_cooperative aspect:default;
        }
    }
}