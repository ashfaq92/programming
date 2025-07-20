// File: /src/agt/scooter1.asl
// Scooter agent. Travels on roads. Handles travel requests assigned by S-CS1.

{ include("$jacamo/templates/common-cartago.asl") }

// Add this belief
emergency(false).

// the travel plan to check emergency status periodically
+!travel(From,To,Customer)[source(s_cs1)] <-
    .print("Scooter: Starting travel from ",From," to ",To);
    // Check if emergency before completing travel
    .wait(1000); // Simulate travel time start
    ?emergency(Status);
    if (Status) {
        .print("Scooter: EMERGENCY! Journey to ", To, " cancelled");
        // REMOVE .fail() - it's causing the error
        // Instead just return early
        .print("Scooter: Returning to base due to emergency")
    } else {
        .wait(3000); // Rest of travel time
        moveVehicle("scooter1",To,Success);
        if (Success) {
            .print("Scooter: Arrived at ",To);
            .send(s_cs1,inform,scooter_done("scooter1",To,Customer));
            .send(Customer,inform,arrived(To))
        } else {
            .print("Scooter: Unable to move");
            .send(s_cs1,inform,scooter_done("scooter1",From,Customer))
        }
    }.


// Add emergency stop handler
+!stop_travel(Reason) <-
    .print("Scooter: EMERGENCY STOP! Reason: ", Reason);
    -+emergency(true).

// Add update route handler  
+!update_route(NewRoute) <-
    .print("Vehicle: Rerouting to ", NewRoute);
    .send(s_sos, inform, status_update("rerouted", NewRoute)).