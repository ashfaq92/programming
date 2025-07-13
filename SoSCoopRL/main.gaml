model main
import "cell_.gaml"
import "nest.gaml"
import "box_.gaml"
import "robot_random.gaml"
import "robot_greedy.gaml"
import "robot_double_cooperative.gaml"
import "robot_saphesia.gaml"

global {
	
	geometry shape <- square(50); 
	
    float seed <- 0.0 parameter:true;
    int max_cycles <- 3000;
    string robot_type_param <- "saphesia";
    
    // Box spawning control
    int box_creation_interval <- 3;
    int box_creation_counter <- 0;
    
    action spawn_nests { 
        create nest with: [ color:rgb("red"), location:cell[15,15].location ];
        create nest with: [ color:rgb("green"), location:cell[35,15].location ];
        create nest with: [ color:rgb("blue"), location:cell[25,32].location ];
    }
    
    action create_robots_with_colors(string robot_type) {
        switch robot_type {
            match "random" { 
                create robot_random number: 30 with: [color::rgb("red")];
                create robot_random number: 30 with: [color::rgb("green")];
                create robot_random number: 30 with: [color::rgb("blue")];
            }
            match "greedy" { 
                create robot_greedy number: 30 with: [color::rgb("red")];
                create robot_greedy number: 30 with: [color::rgb("green")];
                create robot_greedy number: 30 with: [color::rgb("blue")];
            }
            match "double_cooperative" { 
                create robot_double_cooperative number: 30 with: [color::rgb("red")];
                create robot_double_cooperative number: 30 with: [color::rgb("green")];
                create robot_double_cooperative number: 30 with: [color::rgb("blue")];
            }
            match "saphesia" { 
        	    // Create the robots first
			    create robot_double_cooperative number: 30 with: [color::rgb("red")];
			    create robot_double_cooperative number: 30 with: [color::rgb("green")];
			    create robot_double_cooperative number: 30 with: [color::rgb("blue")];
			    
        		
    		    // Then create the component systems to manage them
    			create component_system with: [system_color: rgb("red")];
    			create component_system with: [system_color: rgb("green")];  
    			create component_system with: [system_color: rgb("blue")];
            }
        }
    }
    



	reflex spawn_boxes_at_interval {
	    box_creation_counter <- box_creation_counter + 1;
	    if (box_creation_counter >= box_creation_interval) {
	        list<cell> empty_cells <- cell where (length(agents_inside(each)) = 0);
	        write 'empty cells are ' + length(empty_cells);
	        if (!empty(empty_cells)) {
	            create box_ number: 1 with: [location: one_of(empty_cells).location];
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
    
    
    
    // Data collection lists
    list<float> battery_history <- [];
    list<int> box_count_history <- [];
    list<int> alive_robots_history <- [];
    
    init {
        create simulation with: [seed::seed];
        ask simulation {
            do spawn_nests();
            create box_ number: rnd(10, 15);
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
	            match "random" {
	                if (!empty(robot_random)) {
	                    mean_battery <- mean(robot_random collect each.battery);
	                    alive_robots <- length(robot_random where (each.battery > 0));
	                }
	            }
	            match "greedy" {
	                if (!empty(robot_greedy)) {
	                    mean_battery <- mean(robot_greedy collect each.battery);
	                    alive_robots <- length(robot_greedy where (each.battery > 0));
	                }

					// if we calculate mean battery levels of alive robots only, we get survivorship bias!
//					if (!empty(robot_greedy)) {
//				        list<robot_greedy> alive_robots_list <- robot_greedy where (each.battery > 0);
//				        if (!empty(alive_robots_list)) {
//				            mean_battery <- mean(alive_robots_list collect each.battery);
//				            alive_robots <- length(alive_robots_list);
//				        }
//				    }
	               
	            }
	            match "double_cooperative" {
	                if (!empty(robot_double_cooperative)) {
	                    mean_battery <- mean(robot_double_cooperative collect each.battery);
	                    alive_robots <- length(robot_double_cooperative where (each.battery > 0));
	                }
	                
//	                if (!empty(robot_double_cooperative)) {
//				        list<robot_double_cooperative> alive_robots_list <- robot_double_cooperative where (each.battery > 0);
//				        if (!empty(alive_robots_list)) {
//				            mean_battery <- mean(alive_robots_list collect each.battery);
//				            alive_robots <- length(alive_robots_list);
//				        }
//				    }
	            }
	            match "saphesia" {
	                if (!empty(robot_double_cooperative)) {
	                    mean_battery <- mean(robot_double_cooperative collect each.battery);
	                    alive_robots <- length(robot_double_cooperative where (each.battery > 0));
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
            species robot_random aspect: default;
            species robot_greedy aspect: default;
            species robot_double_cooperative aspect: default;
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