!start.

+!start <-
    .print("WeatherMonitor: Monitoring weather...");
    .wait(5000); // wait 5 seconds
    !disrupt.

+!disrupt <-
    .print("[weather] Severe weather detected! Notifying supervisor...");
    .send(supervisor, tell, weather_disruption).