!start.

+!start : true <-
    .print("========== Customer Journey Request ==========");
    .print("Requesting journey: locX -> locY, deadline: 30");
    .print("==============================================");
    .send(supervisor, tell, request_journey("locX", "locY", 30)).

+request_proposal(Route, ETAs) <-
    .print("========== Route Proposal Received ==========");
    .print("Proposed route: ", Route);
    .print("Estimated times: ", ETAs);
    .print("=============================================");
    .send(supervisor, tell, accept_route).

+journey_aborted(Reason) <-
    .print("[customer] Journey aborted: ", Reason).