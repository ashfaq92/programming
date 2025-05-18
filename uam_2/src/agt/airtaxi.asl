+!assign_to_user(X, Y) <- 
    .print("AirTaxi: Assigned to fly user from", X, "to", Y);
    !check_battery.

+!check_battery <- 
    .my_name(ID);
    .send(s_cs2, achieve, battery_low(ID)).
+!assign_backup(ID) <- .print("AirTaxi: Taking over flight from", ID).
