// File: S-CS2 agent - Air taxi subsystem supervisor

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle incoming taxi requests ONLY if the trip is not already complete
+!kqml_received(Sender, request, taxi_request(From,To,Customer), Mid) : not trip_complete(true) <-
    .print("S-CS2: Received taxi request from ",Sender);
    findVehicle("taxi",From,ID);
    if (ID == "none") {
        .print("S-CS2: No taxi available at ",From)
    } else {
        .print("S-CS2: Assigning taxi ",ID," to the trip");
        .send(taxi1,achieve,travel(From,To,Customer));
        .print("S-CS2: Taxi has been dispatched")
    }.

// Plan to handle taxi completion messages
+!kqml_received(Sender, inform, taxi_done(ID,Loc,Customer), Mid) <-
    .print("S-CS2: Taxi ",ID," delivered customer to ",Loc);
    .send(s_sos,inform,taxi_done(ID,Loc,Customer)).
