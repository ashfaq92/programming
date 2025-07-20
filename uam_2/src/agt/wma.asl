// Weather Manager Agent

+!storm <-
    .print("WMA: Broadcasting storm alert!");
    .send(s_cs1, inform, weather_alert("vertiport", severe));
    .send(s_cs2, inform, weather_alert("vertiport", severe)).   
    // TODO: Should be sent to supervisor or broadcasted generally 

+!start <-
    .print("WMA: Will broadcast storm alert in 3 seconds...");
    .wait(3000); // wait 3 seconds
    !storm.