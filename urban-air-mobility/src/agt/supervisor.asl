// Robust charger artifact lookup
+!ensure_charger_lookup <-
    focus("chargerA");
    lookupArtifact("chargerA", ChargerID);
    -charger_id(_);
    +charger_id(ChargerID).

-!ensure_charger_lookup <-
    .print("[supervisor] Charger artifact not available, retrying in 200ms...");
    .wait(200);
    !ensure_charger_lookup.

// Initial beliefs about available vehicles
available_scooter(scooter1).
available_scooter(scooter2).
available_airtaxi(airtaxi1).
available_airtaxi(airtaxi2).

+request_journey(From, To, Deadline) <-
    // Get first available scooter
    ?available_scooter(S1);
    
    // Robustly get charger artifact and check battery
    !ensure_charger_lookup;
    ?charger_id(ChargerID);
    .local(ChargerID, LocalChargerID); // Store ChargerID in a local variable
    getBatteryLevel(S1, Level)[artifact_id(LocalChargerID)];
    Level > 20;  // Minimum battery threshold
    
    // Get second scooter (must be different from first)
    ?available_scooter(S2) & S2 \== S1;
    
    // Get available air taxi
    ?available_airtaxi(A1);
    
    // Mark vehicles as not available
    -available_scooter(S1);
    -available_scooter(S2);
    -available_airtaxi(A1);
    
    // Print journey details
    .print("========== Journey Coordination ==========");
    .print("Received journey request:");
    .print("  Origin      : ", From);
    .print("  Destination : ", To);
    .print("  Deadline    : ", Deadline);
    .print("------------------------------------------");
    .print("Decomposing journey into segments:");
    .print("  [Leg 1] Mode: ground | ", From, " -> vertiportA | Assigned: ", S1, " (Battery: ", Level, "% )");
    .print("  [Leg 2] Mode: air    | vertiportA -> vertiportB | Assigned: ", A1);
    .print("  [Leg 3] Mode: ground | vertiportB -> ", To, " | Assigned: ", S2);
    .print("------------------------------------------");
    
    // Notify vehicles of their assignments
    .send(S1, tell, plan_segment("ground", From, "vertiportA"));
    .send(A1, tell, plan_segment("air", "vertiportA", "vertiportB"));
    .send(S2, tell, plan_segment("ground", "vertiportB", To));
    
    // Send proposal to customer
    .print("Proposed route: [", S1, ", ", A1, ", ", S2, "] with ETAs [10,20,30]");
    .print("==========================================");
    .send(customer, tell, request_proposal([S1, A1, S2], [10,20,30]));
    
    // Record assignments
    +assigned_leg(1, S1);
    +assigned_leg(2, A1);
    +assigned_leg(3, S2).

+accept_route <-
    ?assigned_leg(1, S1);
    ?assigned_leg(2, A1);
    ?assigned_leg(3, S2);
    .print("Supervisor: Route accepted by customer.");
    +journey_in_progress;
    +current_leg(1);
    .send(S1, tell, execute_journey("leg1")).

// When leg1 is done, start leg2
+leg_finished("leg1") : journey_in_progress <-
    -current_leg(1);
    +current_leg(2);
    ?assigned_leg(2, A1);
    .send(A1, tell, execute_journey("leg2")).

// When leg2 is done, start leg3
+leg_finished("leg2") : journey_in_progress <-
    -current_leg(2);
    +current_leg(3);
    ?assigned_leg(3, S2);
    .send(S2, tell, execute_journey("leg3")).

// When leg3 is done, journey is complete
+leg_finished("leg3") : journey_in_progress <-
    -current_leg(3);
    .print("Supervisor: Journey complete.");
    -journey_in_progress;
    -assigned_leg(1, _);
    -assigned_leg(2, _);
    -assigned_leg(3, _).

+weather_disruption : journey_in_progress & current_leg(1) <-
    .print("Supervisor: Weather disruption received! Aborting all journey legs.");
    ?assigned_leg(1, S1); ?assigned_leg(2, A1); ?assigned_leg(3, S2);
    .send(S1, tell, abort_journey);
    .send(A1, tell, abort_journey);
    .send(S2, tell, abort_journey);
    .send(customer, tell, journey_aborted("Weather disruption"));
    -journey_in_progress;
    -current_leg(1);
    -assigned_leg(1, _);
    -assigned_leg(2, _);
    -assigned_leg(3, _).

+weather_disruption : journey_in_progress & current_leg(2) <-
    .print("Supervisor: Weather disruption received! Aborting leg 2 and 3.");
    ?assigned_leg(2, A1); ?assigned_leg(3, S2);
    .send(A1, tell, abort_journey);
    .send(S2, tell, abort_journey);
    .send(customer, tell, journey_aborted("Weather disruption"));
    -journey_in_progress;
    -current_leg(2);
    -assigned_leg(1, _);
    -assigned_leg(2, _);
    -assigned_leg(3, _).

+weather_disruption : journey_in_progress & current_leg(3) <-
    .print("Supervisor: Weather disruption received! Aborting leg 3.");
    ?assigned_leg(3, S2);
    .send(S2, tell, abort_journey);
    .send(customer, tell, journey_aborted("Weather disruption"));
    -journey_in_progress;
    -current_leg(3);
    -assigned_leg(1, _);
    -assigned_leg(2, _);
    -assigned_leg(3, _).

+release_vehicle(scooter1) <-
    .print("Supervisor: Vehicle scooter1 is now available.");
    +available_scooter(scooter1).

+release_vehicle(scooter2) <-
    .print("Supervisor: Vehicle scooter2 is now available.");
    +available_scooter(scooter2).

+release_vehicle(airtaxi1) <-
    .print("Supervisor: Vehicle airtaxi1 is now available.");
    +available_airtaxi(airtaxi1).

+release_vehicle(airtaxi2) <-
    .print("Supervisor: Vehicle airtaxi2 is now available.");
    +available_airtaxi(airtaxi2).