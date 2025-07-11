/**
* Name: robot_random 
* Author: ashfa
* Tags: CoCaRo
*/


model robot_random

import "robot_base.gaml"

species robot_random parent: robot_base {
    

    reflex target_random_box when: !empty(reachable_boxes) and targeted_box=nil and carried_box=nil and battery > 0 {
        targeted_box <- any(reachable_boxes);
        targeted_box.owner <- self;
    }

 
}


