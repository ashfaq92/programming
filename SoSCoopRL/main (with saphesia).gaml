// Add this to your global section:
global {
    // Create component systems
    init {
        create saphesia_component_system with: [system_color: rgb("red")];
        create saphesia_component_system with: [system_color: rgb("green")];  
        create saphesia_component_system with: [system_color: rgb("blue")];
    }
}

// Add this to your robot_base species (modify the colors_reward_efficiency action):
species robot_base {
    // ... existing attributes ...
    
    // Modified action to be controllable by component systems
    action set_reward_efficiency(rgb color) {
        int not_so_good_reward <- int(reward * counter_efficiency);
        
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
    
    // Allow component systems to override reward efficiency
    action override_reward_efficiency(rgb target_color, int new_reward) {
        colors_reward_efficiency[target_color] <- new_reward;
    }
    
    // ... rest of existing robot_base code ...
}

// In your experiment, create robots with colors:
experiment saphesia_experiment {
    init {
        // Create robots of each color
        create robot_random number: 10 with: [color: rgb("red")];
        create robot_random number: 10 with: [color: rgb("green")];
        create robot_random number: 10 with: [color: rgb("blue")];
        
        // Initialize robot reward efficiency
        ask robot_base {
            do set_reward_efficiency(color);
        }
    }
    
    output {
        display main_display {
            species cell;
            species box_;
            species nest;
            species robot_base aspect: icon;
            species saphesia_component_system;
        }
        
        display system_monitor {
            chart "Component Systems Criticality" type: series {
                data "Red System Criticality" value: first(saphesia_component_system where (each.system_color = rgb("red"))).component_criticality color: rgb("red");
                data "Green System Criticality" value: first(saphesia_component_system where (each.system_color = rgb("green"))).component_criticality color: rgb("green");
                data "Blue System Criticality" value: first(saphesia_component_system where (each.system_color = rgb("blue"))).component_criticality color: rgb("blue");
            }
        }
        
        display robot_survival {
            chart "Robot Survival by System" type: series {
                data "Red Robots" value: length(robot_base where (each.color = rgb("red") and each.battery > 0)) color: rgb("red");
                data "Green Robots" value: length(robot_base where (each.color = rgb("green") and each.battery > 0)) color: rgb("green");
                data "Blue Robots" value: length(robot_base where (each.color = rgb("blue") and each.battery > 0)) color: rgb("blue");
            }
        }
    }
}