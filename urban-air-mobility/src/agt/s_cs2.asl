available_airtaxi(airtaxi1).
available_airtaxi(airtaxi2).

+!allocate_airtaxi(From, To, Deadline, Leg) : available_airtaxi(AirTaxi) <-
    .print("S-CS2: Allocating ", AirTaxi, " for ", From, " -> ", To, " (", Leg, ")");
    .send(AirTaxi, achieve, execute_leg(From, To, Deadline, Leg));
    -available_airtaxi(AirTaxi).

+!allocate_airtaxi(From, To, Deadline, Leg) <-
    .print("S-CS2: No air taxis available for ", From, " -> ", To, " (", Leg, ")").

+leg_done(AirTaxi, From, To, Leg) <-
    .print("S-CS2: ", AirTaxi, " completed ", From, " -> ", To, " (", Leg, ")");
    +available_airtaxi(AirTaxi);
    .send(supervisor, tell, leg_complete(Leg)).