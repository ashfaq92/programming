// File: /src/agt/S-SoS.asl
// System-of-Systems (SoS) supervisor agent for coordinating scooter and air taxi usage.

{ include("$jacamo/templates/common-cartago.asl") }

// Initial beliefs
trip_completed(false).

// Plan to handle travel requests 
+!kqml_received(Sender, achieve, travel_request(From,To), Mid) : not trip_completed(true) <-
    .print("SoS: Planning trip from ",From," to ",To);
    -+tripInfo(From,To,Sender);
    getWeather(W);
    if (W == "storm") {
        .print("SoS: Storm detected. Using scooter for entire trip.");
        .send(s_cs1,request,scooter_request(From,To,Sender))
    } else {
        .print("SoS: Clear weather. Scheduling scooter and air taxi.");
        .send(s_cs1,request,scooter_request(From,"vertiport",Sender));
        .send(s_cs2,request,taxi_request("vertiport",To,Sender))
    }.

// Plan to handle KQML scooter completion
+!kqml_received(Sender, inform, scooter_done(ID,Loc,Customer), Mid) : not trip_completed(true) <-
    ?tripInfo(From,Dest,Customer);
    .print("SoS: Scooter ", ID, " finished at ", Loc);
    if (Loc == Dest) {
        .print("SoS: Customer has arrived at destination by scooter.");
        -+trip_completed(true);
        .send(Customer,inform,trip_complete)
    } else {
        // Only send taxi request if trip is not already complete
        if (not trip_completed(true)) {
            .print("SoS: Requesting air taxi from ", Loc, " to ", Dest);
            .send(s_cs2,request,taxi_request(Loc,Dest,Customer))
        }
    }.

// Plan to handle KQML taxi completion
+!kqml_received(Sender, inform, taxi_done(ID,Loc,Customer), Mid) : not trip_completed(true) <-
    .print("SoS: Taxi ", ID, " finished at ", Loc);
    -+trip_completed(true);
    .send(Customer,inform,trip_complete).


// Plan to handle replan requests from supervisors
+!kqml_received(Sender, request, request_replan(VehicleID, Constraints), Mid) <-
    .print("SoS: Replan requested for ", VehicleID, " due to ", Constraints);
    // For demo: just notify supervisor to update route
    .send(Sender, inform, update_route(VehicleID, "reroute_due_to_storm")).


// Plan to handle vehicle status updates
+!kqml_received(Sender, inform, status_update(Status, Info), Mid) <-
    .print("SoS: Vehicle ", Sender, " status: ", Status, " (", Info, ")").