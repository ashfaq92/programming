// src/agt/s_cs1.asl

// Plan to handle scooter leg
+!plan_scooter_leg(X) <-
    .print("S‑CS1: Planning scooter leg from", X);
    .send(scooter1, achieve, assign_to_user(X));
    .send(scooter2, achieve, reserve_for_landing).  // no parentheses here

// Plan to reroute scooters due to weather
+!reroute_due_to_weather <-
    .print("S‑CS1: Rerouting scooters due to weather.").
