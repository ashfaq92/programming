+!start <-
    .print("========== Customer Journey Request ==========");
    .print("Requesting journey: locX -> locY, deadline: 25");
    .send(supervisor, achieve, journey_request("locX", "locY", 25));
    .print("==============================================").

+journey_complete <-
    .print("Customer: Journey complete!").

!start.


