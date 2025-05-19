// File: /src/agt/taxi1.asl
// Air taxi agent. Travels by air. Handles travel requests assigned by S-CS2.

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle a travel command from S-CS2.
+!travel(From,To,Customer)[source(s_cs2)] <-
    .print("Taxi: Starting travel from ",From," to ",To);
    moveVehicle("taxi1",To,Success);
    if ( Success ) {
        .print("Taxi: Arrived at ",To);
        .send(s_cs2,inform,taxi_done("taxi1",To,Customer));
        .send(Customer,inform,arrived(To))
    } else {
        .print("Taxi: Unable to move");
        .send(s_cs2,inform,taxi_done("taxi1",From,Customer))
    }.
