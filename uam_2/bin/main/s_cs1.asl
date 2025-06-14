// File: /src/agt/S-CS1.asl
// Scooter subsystem supervisor agent. Manages scooter availability and assignments.

{ include("$jacamo/templates/common-cartago.asl") }

// Plan to handle incoming request messages
+!kqml_received(Sender, request, scooter_request(From,To,Customer), Mid) <-
    .print("S-CS1: Received scooter request from ",Sender);
    .send(s_sos, achieve, check_emergency(self));
    .wait(500); // Wait for response
    !dispatch_if_safe(From, To, Customer).

// Only dispatch if no emergency
+!dispatch_if_safe(From, To, Customer) : not emergency_status(true) <-
    .print("S-CS1: Conditions safe, finding scooter...");
    findVehicle("scooter",From,ID);
    if (ID == "none") {
        .print("S-CS1: No scooter available at ",From)
    } else {
        .print("S-CS1: Assigning scooter ",ID," to the trip.");
        .send(scooter1,achieve,travel(From,To,Customer));
        .print("S-CS1: Scooter ",ID," has been dispatched.")
    }.

// Emergency condition detected
+!dispatch_if_safe(From, To, Customer) : emergency_status(true) <-
    .print("S-CS1: EMERGENCY CONDITIONS! Cannot dispatch scooter.").

// Handle emergency status update from SoS
+!kqml_received(Sender, tell, emergency_status(Status), Mid) <-
    -+emergency_status(Status);
    .print("S-CS1: Emergency status updated to: ", Status);
    if (Status) {
        .print("S-CS1: EMERGENCY DETECTED! Cancelling pending scooter assignments.")
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

// Add to both supervisor files - emergency vehicle recall
+!recall_vehicles(Reason) <-
    .print("Supervisor: EMERGENCY RECALL - ", Reason);
    // Send stop command to vehicles
    .send(scooter1, achieve, stop_travel(Reason)).  // For s_cs1
    // OR 
    .send(taxi1, achieve, stop_travel(Reason)).     // For s_cs2