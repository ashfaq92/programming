// File: S-CS2 agent - Air taxi subsystem supervisor

{ include("$jacamo/templates/common-cartago.asl") }

// Before dispatching, check emergency status
+!kqml_received(Sender, request, taxi_request(From,To,Customer), Mid) <-
    .print("S-CS2: Received taxi request from ",Sender);
    .send(s_sos, achieve, check_emergency(self));
    .wait(500); // Wait for response
    !dispatch_if_safe(From, To, Customer).

// Only dispatch if no emergency
+!dispatch_if_safe(From, To, Customer) : not emergency_status(true) <-
    .print("S-CS2: Conditions safe, finding taxi...");
    findVehicle("taxi",From,ID);
    if (ID == "none") {
        .print("S-CS2: No taxi available at ",From)
    } else {
        .print("S-CS2: Assigning taxi ",ID," to the trip");
        .send(taxi1,achieve,travel(From,To,Customer));
        .print("S-CS2: Taxi has been dispatched")
    }.

// Emergency condition detected
+!dispatch_if_safe(From, To, Customer) : emergency_status(true) <-
    .print("S-CS2: EMERGENCY CONDITIONS! Cannot dispatch taxi.").

// Handle emergency status update from SoS
+!kqml_received(Sender, tell, emergency_status(Status), Mid) <-
    -+emergency_status(Status);
    .print("S-CS2: Emergency status updated to: ", Status).

// Plan to handle weather alerts
+!kqml_received(Sender, inform, weather_alert(Zone, Severity), Mid) <-
    .print("Supervisor: Weather alert received for ", Zone, " (", Severity, ")");
    .send(s_sos, request, request_replan(self, [storm(Zone,Severity)])).

// Plan to handle route updates for vehicles
+!kqml_received(Sender, inform, update_route(VehicleID, NewRoute), Mid) <-
    .print("Supervisor: Updating route for ", VehicleID, " to ", NewRoute);
    .send(VehicleID, achieve, update_route(NewRoute)).
    
+!update_route(NewRoute) <-
    .print("S-CS2: Ignoring update_route for self.").

// Emergency vehicle recall - for taxi only
+!recall_vehicles(Reason) <-
    .print("Supervisor: EMERGENCY RECALL - ", Reason);
    .send(taxi1, achieve, stop_travel(Reason)).

// Handle taxi completion
+!kqml_received(Sender, inform, taxi_done(ID,Loc,Customer), Mid) <-
    .print("S-CS2: Taxi ",ID," delivered customer to ",Loc);
    .send(s_sos,inform,taxi_done(ID,Loc,Customer)).