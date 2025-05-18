// Wait for user request
+!handle_request(X, Y) <- 
    .print("S-SoS: Handling request from", X, "to", Y);
    !analyze_request(X, Y).

+!analyze_request(X, Y) <- 
    .print("S-SoS: Decomposing task and assigning to subsystems...");
    .send(s_cs1, achieve, plan_scooter_leg(X));
    .send(s_cs2, achieve, plan_air_taxi_leg(X, Y)).

+!update_plan(Status) <- .print("S-SoS: Plan updated:", Status).
