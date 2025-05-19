// File: /src/agt/S-CS1.asl
// Scooter subsystem supervisor agent. Manages scooter availability and assignments.

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle incoming request messages
+!kqml_received(Sender, request, scooter_request(From,To,Customer), Mid) <-
    .print("S-CS1: Received scooter request from ",Sender);
    findVehicle("scooter",From,ID);
    if (ID == "none") {
        .print("S-CS1: No scooter available at ",From)
    } else {
        .print("S-CS1: Assigning scooter ",ID," to the trip.");
        .send(scooter1,achieve,travel(From,To,Customer));
        .print("S-CS1: Scooter ",ID," has been dispatched.")
    }.

// Plan to handle scooter completion messages
+!kqml_received(Sender, inform, scooter_done(ID,Loc,Customer), Mid) <-
    .print("S-CS1: Scooter ",ID," delivered customer to ",Loc);
    .send(s_sos,inform,scooter_done(ID,Loc,Customer)).
