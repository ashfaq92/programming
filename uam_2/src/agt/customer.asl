// File: /src/agt/c1.asl
// Customer agent requesting a trip from suburban to downtown

{ include("$jacamo/templates/common-cartago.asl") }

// Initial beliefs
trip_complete(false).

// Plan to request trip
+!travel : not trip_complete(true) <-
    .print("Customer: Requesting trip from suburban to downtown");
    .send(s_sos,achieve,travel_request(suburban,downtown)).

// Plan to handle KQML arrival notifications
+!kqml_received(Sender, inform, arrived(Location), Mid) : not trip_complete(true) <-
    .print("Customer: Arrived at ", Location, "!").

// Plan to handle KQML trip completion
+!kqml_received(Sender, inform, trip_complete, Mid) : not trip_complete(true) <-
    -+trip_complete(true);
    .print("Customer: Trip completed successfully! Thank you.").
