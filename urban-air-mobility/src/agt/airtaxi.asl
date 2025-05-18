+plan_segment(Mode, From, To) <-
    .print("Assigned to segment: ", Mode, ", ", From, " -> ", To).

+execute_journey(Leg) <-
    .print("Executing journey leg ", Leg);
    .my_name(A);
    .send(supervisor, tell, leg_finished(Leg));
    .send(supervisor, tell, release_vehicle(A)).

+abort_journey <-
    .print("AirTaxi: Aborting journey due to weather disruption.").