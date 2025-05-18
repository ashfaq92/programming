+!plan_air_taxi_leg(X, Y) <- 
    .print("S-CS2: Planning air-taxi leg from", X, "to", Y);
    .send(airtaxi1, achieve, assign_to_user(X, Y)).

+!battery_low(ID) <- 
    .print("S-CS2: Battery low for", ID, ". Reassigning...");
    .send(airtaxi2, achieve, assign_backup(ID)).
