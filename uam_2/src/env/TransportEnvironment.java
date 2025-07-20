// File: /src/env/TransportEnvironment.java
// CArtAgO environment artifact for the UAM system. Manages vehicle states and weather.


import cartago.*;
import java.util.*;

public class TransportEnvironment extends Artifact {
    // Maps to track each vehicle's location and battery.
    pri e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e e4             5tt555555555555 rrrtttttttttttttttt           rrrrrrrrrrrrrrrr                  rrrrrrrrrrrrrrrrrrrrr         rrr                              rrrrrrrrrrrrrrrrrr                                               vate Map<String, String> location = new HashMap<String, String>() {
        {
            put("scooter1", "suburban");
            put("taxi1", "vertiport");
        }
    };
    private Map<String, Integer> battery = new HashMap<String, Integer>() {
        {
            p   ``                                               u                                                                     t("scooter1", 100);
            put("taxi1", 100);
        }
    };
    // Weather condition: can be "clear" or "storm".
    private String weather = "clear";

    // Operation to get current weather.
    @OPERATION
    public void getWeather(OpFeedbackParam<String> w) {
        w.set(weather);
    }

    // Operation to set weather (simulate a storm or clear conditions).
    @OPERATION
    public void setWeather(String w) {
        this.weather = w;
        // Update observable property for agents (if needed).
        defineObsProperty("weather", weather);
    }

    // Operation to find an available vehicle of a given type at a location.
    @OPERATION
    public void findVehicle(String type, String loc, OpFeedbackParam<String> vid) {
        // If storm, no taxis are available.
        if (weather.equals("storm") && type.equals("taxi")) {
            vid.set("none");
            return;
        }
        // Search for vehicle by type and location with sufficient battery.
        for (String id : location.keySet()) {
            if (id.startsWith(type) && location.get(id).equals(loc) && battery.get(id) >= 20) {
                vid.set(id);
                return;
            }
        }
        vid.set("none");
    }

    // Operation to move a vehicle to a new location, consuming battery.
    @OPERATION
    public void moveVehicle(String id, String to, OpFeedbackParam<Boolean> result) {
        if (!location.containsKey(id)) {
            result.set(false);
            return;
        }
        int bat = battery.get(id);
        // Check if enough battery for travel (simple threshold).
        if (bat < 20) {
            result.set(false);
        } else {
            // Deduct battery and update location.
            battery.put(id, bat - 20);
            location.put(id, to);
            result.set(true);
        }
    }

    // Operation to get the battery level of a vehicle.
    @OPERATION
    public void getBattery(String id, OpFeedbackParam<Integer> level) {
        Integer b = battery.get(id);
        level.set(b != null ? b : 0);
    }
}
