/**
 * SApHESIA System of Systems Implementation
 * Each color group becomes a component-system that can cooperate
 */

species component_system skills: [communicating] {
    rgb system_color;
    list<robot_base> robots update: robot_base where (each.color = system_color and !dead(each));
    list<robot_base> dying_robots update: robots where (each.battery < max_battery/3);
    list<component_system> linked_systems;
    
    // SApHESIA model attributes
    int component_criticality update: length(dying_robots);
    float criticality_ratio update: (length(robots) = 0) ? 0.0 : (length(dying_robots) / length(robots));
    bool needs_help update: criticality_ratio > 0.5; // More than half robots dying
    bool is_helping <- false;
    
    init {
        linked_systems <- component_system - self;
    }
    
    // Functionality: Help another component-system by changing robot perceptions
    action help_system(component_system target_system) {
        write "Component " + system_color + " helping " + target_system.system_color;
        
        ask robots {
            // Make our robots treat target system's boxes as valuable as our own
            colors_reward_efficiency[target_system.system_color] <- reward;
            write self.name + " now values " + target_system.system_color + " boxes highly";
        }
        is_helping <- true;
    }
    
    action stop_helping(component_system target_system) {
        write "Component " + system_color + " stopping help to " + target_system.system_color;
        
        ask robots {
            // Reset reward efficiency to normal
            colors_reward_efficiency[target_system.system_color] <- reduced_reward;
        }
        is_helping <- false;
    }
    
    // Calculate anticipated criticality if we help another system
    float calculate_anticipated_criticality(component_system target_system) {
        // Estimate how our criticality would change if we help
        // Helping means our robots will spend time on other colors' boxes
        float efficiency_loss <- 0.3; // Assume 30% efficiency loss when helping
        float anticipated_dying <- length(dying_robots) * (1.0 + efficiency_loss);
        return min(anticipated_dying, length(robots));
    }
    
    aspect default {
        draw circle(5) color: system_color at: {world.shape.width * (system_color = rgb("red") ? 0.1 : (system_color = rgb("green") ? 0.5 : 0.9)), 10};
        draw string("System " + system_color + "\nRobots: " + length(robots) + "\nDying: " + length(dying_robots)) 
            color: rgb("white") size: 1.0 at: {world.shape.width * (system_color = rgb("red") ? 0.1 : (system_color = rgb("green") ? 0.5 : 0.9)), 20};
    }
}

species saphesia_component_system parent: component_system {
    
    // Request help when in critical state
    reflex request_help when: needs_help and !is_helping {
        write "System " + system_color + " requesting help - " + length(dying_robots) + "/" + length(robots) + " robots dying";
        
        loop target_system over: linked_systems {
            if (!target_system.is_helping and target_system.criticality_ratio < 0.3) {
                do send with: [
                    receivers: target_system,
                    protocol: 'fipa-request',
                    performative: 'request',
                    content: ['help_request', component_criticality, system_color]
                ];
            }
        }
    }
    
    // Process help requests using cooperative decision algorithm
    reflex process_help_requests when: !empty(requests) {
        loop request over: requests {
            if (request.content[0] = 'help_request') {
                int requester_criticality <- int(request.content[1]);
                rgb requester_color <- rgb(request.content[2]);
                component_system requester <- first(linked_systems where (each.system_color = requester_color));
                
                if (requester != nil) {
                    // Apply cooperative decision algorithm from paper
                    float my_anticipated_crit <- calculate_anticipated_criticality(requester);
                    float their_current_crit <- float(requester_criticality);
                    
                    // Help if they are more critical and we won't become too critical
                    if (their_current_crit > component_criticality and my_anticipated_crit < their_current_crit) {
                        do agree with: [message: request, content: ['help_accepted']];
                        do help_system(requester);
                        write "System " + system_color + " agreed to help " + requester_color + 
                              " (their_crit: " + their_current_crit + ", my_anticipated: " + my_anticipated_crit + ")";
                    } else {
                        do refuse with: [message: request, content: ['help_refused']];
                        write "System " + system_color + " refused to help " + requester_color + 
                              " (their_crit: " + their_current_crit + ", my_anticipated: " + my_anticipated_crit + ")";
                    }
                }
            }
        }
    }
    
    // Handle responses to our help requests
    reflex handle_help_responses when: !empty(agrees) or !empty(refuses) {
        loop agree over: agrees {
            if (agree.content[0] = 'help_accepted') {
                write "System " + system_color + " received help from " + agree.sender;
            }
        }
        
        loop refuse over: refuses {
            if (refuse.content[0] = 'help_refused') {
                write "System " + system_color + " help refused by " + refuse.sender;
            }
        }
    }
    
    // Stop helping when situation improves
    reflex stop_helping_when_stable when: is_helping and criticality_ratio < 0.2 {
        write "System " + system_color + " situation improved, stopping help";
        
        ask linked_systems {
            if (myself.is_helping) {
                myself.stop_helping(self);
            }
        }
    }
    
    // Monitor and report system state
    reflex monitor_system {
        if (cycle mod 50 = 0) {
            write "=== System " + system_color + " Status ===";
            write "Total robots: " + length(robots);
            write "Dying robots: " + length(dying_robots);
            write "Criticality ratio: " + criticality_ratio;
            write "Needs help: " + needs_help;
            write "Is helping: " + is_helping;
            write "================================";
        }
    }
}