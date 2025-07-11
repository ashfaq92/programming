/**
* Name: box
* Based on the internal empty template. 
* Author: ashfa
* Tags: 
*/



model box_

import "cell_.gaml"

species box_ {
	list<rgb> colors <- [rgb("red"), rgb("green"), rgb("blue")];
	rgb color;
	agent owner <- nil;
	//cell myCell update: cell first_with (each overlaps self);
	// cell myCell <- cell(location);  // fastest
	// cell myCell <- cell closest_to self;
	cell myCell update: cell closest_to self;	
	
	
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