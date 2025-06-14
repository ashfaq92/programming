// File: /src/agt/S-SoS.asl
// System-of-Systems (SoS) supervisor agent for coordinating scooter and air taxi usage.

{ include("$jacamo/templates/common-cartago.asl") }

// Initial beliefs
trip_completed(false).
emergency(false).  // <-- Add this belief

// Handle travel request
+!kqml_received(Sender, achieve, travel_request(From,To), Mid) : not trip_completed(true) <-
    .print("SoS: Planning trip from ", From, " to ", To);
    +tripInfo(From,To,Sender);
    -+emergency(false);  // Reset emergency status at start
    .send(s_cs1, request, scooter_request(From, "vertiport", Sender)).

// Handle scooter done
+!kqml_received(Sender, inform, scooter_done(ID,Loc,Customer), Mid) : emergency(true) <-
    .print("SoS: Ignoring scooter completion - emergency active").

// Handle scooter done (original plan for non-emergency)
+!kqml_received(Sender, inform, scooter_done(ID,Loc,Customer), Mid) : not trip_completed(true) & not emergency(true) <-
    ?tripInfo(From,Dest,Customer);
    .print("SoS: Scooter ", ID, " finished at ", Loc);
    if (Loc == Dest) {
        .print("SoS: Customer has arrived at destination by scooter.");
        +trip_completed(true);
        .send(Customer, inform, trip_complete)
    } else {
        .print("SoS: Requesting air taxi from ", Loc, " to ", Dest);
        .send(s_cs2, request, taxi_request(Loc, Dest, Customer))
    }.

// Handle taxi done
+!kqml_received(Sender, inform, taxi_done(ID,Loc,Customer), Mid) : not trip_completed(true) & not emergency(true) <-
    .print("SoS: Taxi ", ID, " finished at ", Loc);
    +trip_completed(true);
    .send(Customer, inform, trip_complete).

// NEW: Check if emergency is active
+!check_emergency(Reply) <-
    ?emergency(Status);
    .send(Reply, tell, emergency_status(Status)).

// Storm cancellation - WITH vehicle recall
+!kqml_received(Sender, request, request_replan(VehicleID, [storm(Zone,Severity)]), Mid) <-
    .print("SoS: EMERGENCY! Storm detected at ", Zone, ". CANCELLING ALL TRAVEL!");
    -+emergency(true);
    +trip_completed(true);
    
    // ADD THESE LINES - send recall commands to both supervisors
    .send(s_cs1, achieve, recall_vehicles("Storm emergency"));
    .send(s_cs2, achieve, recall_vehicles("Storm emergency"));
    
    .send(c1, inform, trip_cancelled("Storm emergency")).

// Handle vehicle status updates
+!kqml_received(Sender, inform, status_update(Status, Info), Mid) <-
    .print("SoS: Vehicle ", Sender, " status: ", Status, " (", Info, ")").
