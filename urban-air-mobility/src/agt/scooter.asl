+!execute_leg(From, To, Deadline, Leg) <-
    .print(self, ": Executing scooter leg ", From, " -> ", To, " (", Leg, ")");
    .wait(1000);
    .print(self, ": Completed leg from ", From, " to ", To);
    .send(s_cs1, tell, leg_done(self, From, To, Leg)).