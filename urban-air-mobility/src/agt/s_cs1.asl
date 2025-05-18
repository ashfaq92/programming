// Initial beliefs about available scooters
available_scooter(scooter1).
available_scooter(scooter2).

+!allocate_scooter(From, To, Deadline, Leg) : available_scooter(Scooter) <-
    .print("S-CS1: Allocating ", Scooter, " for ", From, " -> ", To, " (", Leg, ")");
    .send(Scooter, achieve, execute_leg(From, To, Deadline, Leg));
    -available_scooter(Scooter).

+!allocate_scooter(From, To, Deadline, Leg) <-
    .print("S-CS1: ERROR - No scooters available for ", From, " -> ", To, " (", Leg, ")").

+leg_done(Scooter, From, To, Leg) <-
    .print("S-CS1: ", Scooter, " completed ", From, " -> ", To, " (", Leg, ")");
    ?available_scooter(Scooter) // Check if belief exists
        : true 
        <- true  // Do nothing if exists
        : not available_scooter(Scooter) 
        <- +available_scooter(Scooter); // Add only if not exists
    .send(supervisor, tell, leg_complete(Leg)).