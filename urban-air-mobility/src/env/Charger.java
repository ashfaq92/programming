import cartago.*;

public class Charger extends Artifact {
    void init(String id) {
        defineObsProperty("id", id);
        defineObsProperty("available", true);
    }

    @OPERATION
    void reserve() {
        ObsProperty avail = getObsProperty("available");
        if (avail.booleanValue()) {
            avail.updateValue(false);
        } else {
            failed("Already reserved");
        }
    }

    @OPERATION
    void release() {
        getObsProperty("available").updateValue(true);
    }
}