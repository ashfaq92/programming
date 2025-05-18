


import cartago.Artifact;
import cartago.OPERATION;

public class Vertiport extends Artifact {
    private String name;

    public void init(String name) {
        this.name = name;
        defineObsProperty("capacity", 2); // stub: 2 slots
    }

    @OPERATION
    public void reserveSlot() {
        System.out.println("Vertiport " + name + ": slot reserved.");
    }
}