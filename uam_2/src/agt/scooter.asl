// File: /src/agt/scooter1.asl
// Scooter agent. Travels on roads. Handles travel requests assigned by S-CS1.

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle a travel command from S-CS1.
+!travel(From,To,Customer)[source(s_cs1)] <-
    .print("Scooter: Starting travel from ",From," to ",To);
    .wait(4000); // <-- Add this line to simulate travel time (4 seconds)
    moveVehicle("scooter1",To,Success);
    if (Success) {
        .print("Scooter: Arrived at ",To);
        .send(s_cs1,inform,scooter_done("scooter1",To,Customer));
        .send(Customer,inform,arrived(To))
    } else {
        .print("Scooter: Unable to move");
        .send(s_cs1,inform,scooter_done("scooter1",From,Customer))
    }.


// Plan to handle route update
+!update_route(NewRoute) <-
    .print("Vehicle: Rerouting to ", NewRoute);
    .send(s_sos, inform, status_update("rerouted", NewRoute)).