package devices;

import cartago.*;

public class HVAC extends Artifact {

    void init(double initialTemp) {
        defineObsProperty("state", "idle");
        defineObsProperty("temperature", initialTemp); // careful with the type
        log("Temperature: " + getObsProperty("temperature").doubleValue());
    }

    @OPERATION
    void startHeating() {
        getObsProperty("state").updateValue("heating");
        execInternalOp("updateTemperatureProc", 0.5);
    }

    @OPERATION
    void startCooling() {
        getObsProperty("state").updateValue("cooling");
        execInternalOp("updateTemperatureProc", -0.5);
    }

    @OPERATION
    void stopAirConditioner() {
        getObsProperty("state").updateValue("idle");
    }

    @INTERNAL_OPERATION
    void updateTemperatureProc(double step) {
        ObsProperty tempProp = getObsProperty("temperature");
        ObsProperty state = getObsProperty("state");
        while (!state.stringValue().equals("idle")) {
            double temp = tempProp.doubleValue();
            tempProp.updateValue(temp + step);
            log("Temperature: " + temp);
            this.await_time(1000);
        }
    }
}
