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


// Plan to handle weather alerts
+!kqml_received(Sender, inform, weather_alert(Zone, Severity), Mid) <-
    .print("Supervisor: Weather alert received for ", Zone, " (", Severity, ")");
    // For demo: always request replan for the current vehicle
    .send(s_sos, request, request_replan(self, [storm(Zone,Severity)])).


// Plan to handle route update from SoS
+!kqml_received(Sender, inform, update_route(VehicleID, NewRoute), Mid) <-
    .print("Supervisor: Updating route for ", VehicleID, " to ", NewRoute);
    .send(scooter1, achieve, update_route(NewRoute)).

// Plan to ignore self route updates
+!update_route(NewRoute) <-
    .print("S-CS1: Ignoring update_route for self.").