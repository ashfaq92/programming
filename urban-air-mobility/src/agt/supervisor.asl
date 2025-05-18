+!journey_request(From, To, Deadline) <-
    .print("Supervisor: Received journey request from ", From, " to ", To);
    +journey_params(From, To, Deadline);
    .print("Supervisor: Initiating first leg - scooter to vertiport");
    .send(s_cs1, achieve, allocate_scooter(From, "vertiportA", Deadline, leg1)).

+leg_complete(leg1) <-
    ?journey_params(_, To, Deadline);
    .print("Supervisor: First leg complete. Initiating air-taxi transfer");
    .send(s_cs2, achieve, allocate_airtaxi("vertiportA", "vertiportB", Deadline, leg2)).

+leg_complete(leg2) <-
    ?journey_params(_, To, Deadline);
    .print("Supervisor: Air-taxi leg complete. Initiating final scooter leg");
    .send(s_cs1, achieve, allocate_scooter("vertiportB", To, Deadline, leg3)).

+leg_complete(leg3) <-
    .print("Supervisor: Full journey completed successfully");
    .send(customer, tell, journey_complete).