model main

import "cell_.gaml"
import "nest.gaml"
import "box_.gaml"
import "robot_greedy.gaml"
import "robot_cooperative.gaml"
import "robot_saphesia.gaml" 

global {
	
	geometry shape <- square(50); 
	
    float randSeed <- 42.0;
    int max_cycles <- 1000;
    string robot_type_param <- "greedy";
    
    // Box spawning control
    int box_creation_interval <- 3;
    int box_creation_counter <- 0;
    
    action spawn_nests { 
        create nest with: [ color:rgb("red"), location:cell[15,15].location ];
        create nest with: [ color:rgb("green"), location:cell[35,15].location ];
        create nest with: [ color:rgb("blue"), location:cell[25,32].location ];
    }
    
    action create_robots_with_colors(string robot_type) {
        // Get available spawn locations (avoiding nests)
        list<cell> spawn_cells <- cell where (length(agents_inside(each)) = 0);
        
        switch robot_type {
            match "greedy" { 
                // Create robots with deterministic locations
                loop i from: 0 to: 29 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_greedy number: 1 with: [color::rgb("red"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 30 to: 59 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_greedy number: 1 with: [color::rgb("green"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 60 to: 89 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_greedy number: 1 with: [color::rgb("blue"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
            }
            match "cooperative" { 
                // Create robots with deterministic locations
                loop i from: 0 to: 29 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("red"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 30 to: 59 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("green"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 60 to: 89 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("blue"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
            }
            match "saphesia" { 
        	    // Create the robots first with deterministic locations
        	    loop i from: 0 to: 29 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("red"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 30 to: 59 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("green"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
                loop i from: 60 to: 89 {
                    int cell_index <- int((randSeed + i * 23) * 37) mod length(spawn_cells);
                    create robot_cooperative number: 1 with: [color::rgb("blue"), location::spawn_cells[cell_index].location];
                    remove index: cell_index from: spawn_cells;
                }
			    
        		
    		    // Then create the component systems to manage them
    			create component_system with: [system_color: rgb("red")];
    			create component_system with: [system_color: rgb("green")];  
    			create component_system with: [system_color: rgb("blue")];
            }
        }
    }
    
    // Fixed deterministic box spawning with proper distribution
    action spawn_initial_boxes {
        // Use a fixed number instead of rnd() for deterministic behavior
        int initial_box_count <- 12; // Fixed number instead of rnd(10, 15)
        
        list<cell> empty_cells <- cell where (length(agents_inside(each)) = 0);
        
        // Create boxes distributed across the grid deterministically
        loop i from: 0 to: min(initial_box_count - 1, length(empty_cells) - 1) {
            // Use seed-based deterministic distribution instead of sequential placement
            int cell_index <- int((randSeed + i * 17) * 31) mod length(empty_cells);
            create box_ number: 1 with: [location: empty_cells[cell_index].location];
            
            // Remove the used cell to avoid duplicates
            remove index: cell_index from: empty_cells;
        }
    }

	reflex spawn_boxes_at_interval {
	    box_creation_counter <- box_creation_counter + 1;
	    if (box_creation_counter >= box_creation_interval) {
	        list<cell> empty_cells <- cell where (length(agents_inside(each)) = 0);
	        write 'empty cells are ' + length(empty_cells);
	        if (!empty(empty_cells)) {
	            // For deterministic behavior, use indexed access instead of one_of()
	            int cell_index <- rnd(length(empty_cells) - 1);
	            create box_ number: 1 with: [location: empty_cells[cell_index].location];
	        }
	        box_creation_counter <- 0;
	    }
	}
    
    reflex stop_simulation when: cycle >= max_cycles {
        write "Simulation completed at cycle " + cycle;
        ask world { do pause; }
    }
    
   
}

// ===== MODULAR EXPERIMENT: CONFIGURABLE ROBOT TYPE =====
experiment robot_analysis type: gui {
    // Set seed directly as a parameter
    parameter "Random Seed" var: randSeed <- 42.0;
    
    
    // Data collection lists
    list<float> battery_history <- [];
    list<int> box_count_history <- [];
    list<int> alive_robots_history <- [];
    
    init {
        // Explicitly set the seed before creating simulation
        seed <- randSeed;
        write "Using seed: " + seed;
        
        create simulation with: [seed::seed];
        ask simulation {
            // Set seed again in simulation context to ensure consistency
            seed <- randSeed;
            write "Simulation seed set to: " + seed;
            
            do spawn_nests();
            // Use deterministic box spawning instead of random
            do spawn_initial_boxes();
            do create_robots_with_colors(robot_type_param);
        }
    }
    
  
    reflex collect_and_export_data when: cycle mod 10 = 0 {
	    ask simulation {
	        // Variables to store current metrics
	        float mean_battery <- 0.0;
	        int alive_robots <- 0;
	        int box_count <- length(box_);
	        
	        // Get metrics based on robot type
	        switch robot_type_param {
	            match "greedy" {
	                if (!empty(robot_greedy)) {
	                    mean_battery <- mean(robot_greedy collect each.battery);
	                    alive_robots <- length(robot_greedy where (each.battery > 0));
	                }
	            }
	            match "cooperative" {
	                if (!empty(robot_cooperative)) {
	                    mean_battery <- mean(robot_cooperative collect each.battery);
	                    alive_robots <- length(robot_cooperative where (each.battery > 0));
	                }

	            }
	            match "saphesia" {
	                if (!empty(robot_cooperative)) {
	                    mean_battery <- mean(robot_cooperative collect each.battery);
	                    alive_robots <- length(robot_cooperative where (each.battery > 0));
	                }
	            }
	        }
	        
	        // Store data in histories
	        add mean_battery to: myself.battery_history;
	        add box_count to: myself.box_count_history;
	        add alive_robots to: myself.alive_robots_history;
	        
	        // Export all data to CSV (APPEND MODE)
	        save [cycle, mean_battery, box_count, alive_robots, robot_type_param] 
	             //to: "results/complete_analysis.csv" type: csv header: false rewrite: false;
	             to: "../results/data_gama.csv" format: "csv" header: false rewrite: false;
	            
	    }
	}
    
    output {
        display "Robot Simulation" type: opengl {
            grid cell border: #black;
            species cell aspect: default;
            species nest aspect: default;
            species box_ aspect: default;
            species robot_greedy aspect: default;
            species robot_cooperative aspect: default;
            //species component_system aspect: default;
        }
        
        // Chart 1: Battery Level Over Time
        display "Battery Level Analysis" type: 2d {
            chart "Mean Battery Level" type: series {
                data "Battery" value: length(battery_history) > 0 ? battery_history[length(battery_history) - 1] : 0.0 color: #blue;
            }
        }
        
        // Chart 2: Box Count Over Time  
        display "Box Count Analysis" type: 2d {
            chart "Boxes in Environment" type: series {
                data "Boxes" value: length(box_count_history) > 0 ? box_count_history[length(box_count_history) - 1] : 0 color: #red;
            }
        }
        
        // Chart 3: Alive Robots Over Time
        display "Robot Survival Analysis" type: 2d {
            chart "Living Robots" type: series {
                data "Alive" value: length(alive_robots_history) > 0 ? alive_robots_history[length(alive_robots_history) - 1] : 0 color: #green;
            }
        }
        
        // Monitors
        monitor "Robot Type" value: robot_type_param;
        monitor "Cycle" value: cycle;
        monitor "Living Robots" value: length(alive_robots_history) > 0 ? alive_robots_history[length(alive_robots_history) - 1] : 0;
        monitor "Mean Battery" value: length(battery_history) > 0 ? battery_history[length(battery_history) - 1] : 0;
        monitor "Boxes in Environment" value: length(box_count_history) > 0 ? box_count_history[length(box_count_history) - 1] : 0;
    }
}