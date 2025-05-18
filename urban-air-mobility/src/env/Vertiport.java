import cartago.*;

public class Vertiport extends Artifact {
    void init(String id) {
        defineObsProperty("id", id);
        defineObsProperty("slot_available", true);
    }

    @OPERATION
    void reserve_slot() {
        ObsProperty slot = getObsProperty("slot_available");
        if (slot.booleanValue()) {
            slot.updateValue(false);
        } else {
            failed("Slot not available");
        }
    }

    @OPERATION
    void release_slot() {
        getObsProperty("slot_available").updateValue(true);
    }
}