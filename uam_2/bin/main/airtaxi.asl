// File: /src/agt/taxi1.asl
// Air taxi agent. Travels by air. Handles travel requests assigned by S-CS2.

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle a travel command from S-CS2.
+!travel(From,To,Customer)[source(s_cs2)] <-
    .print("Taxi: Starting travel from ",From," to ",To);
    .wait(5000); // <-- Add this line to simulate travel time (5 seconds)
    moveVehicle("taxi1",To,Success);
    if ( Success ) {
        .print("Taxi: Arrived at ",To);
        .send(s_cs2,inform,taxi_done("taxi1",To,Customer));
        .send(Customer,inform,arrived(To))
    } else {
        .print("Taxi: Unable to move");
        .send(s_cs2,inform,taxi_done("taxi1",From,Customer))
    }.



// Plan to handle route update
+!update_route(NewRoute) <-
    .print("Vehicle: Rerouting to ", NewRoute);
    .send(s_sos, inform, status_update("rerouted", NewRoute)).

// Add to both vehicle files - emergency stop
+!stop_travel(Reason) <-
    .print("Vehicle: EMERGENCY STOP! Reason: ", Reason);
    // Clear any travel in progress
    .drop_all_intentions.