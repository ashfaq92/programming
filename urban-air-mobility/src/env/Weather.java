import cartago.*;

public class Weather extends Artifact {
    void init() {
        defineObsProperty("condition", "clear");
    }

    @OPERATION
    void setWeather(String condition) {
        getObsProperty("condition").updateValue(condition);
    }
}