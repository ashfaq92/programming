import cartago.*;

public class Charger extends Artifact {
    private String location;
    
    void init(String loc) {
        this.location = loc;
        defineObsProperty("location", loc);
        defineObsProperty("battery_level", 100);
    }
    
    @OPERATION
    void getBatteryLevel(String vehicle, OpFeedbackParam<Integer> level) {
        // Simulate different battery levels for different vehicles
        switch(vehicle) {
            case "scooter1":
                level.set(85);
                break;
            case "scooter2":
                level.set(75);
                break;
            default:
                level.set(100);
        }
        log("Checked battery level for " + vehicle + " at charger " + location);
    }
}