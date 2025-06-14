+!execute_leg(From, To, Deadline, Leg) <-
    .print(self, ": Executing air-taxi leg ", From, " -> ", To, " (", Leg, ")");
    .wait(1000);
    .send(s_cs2, tell, leg_done(self, From, To, Leg)).