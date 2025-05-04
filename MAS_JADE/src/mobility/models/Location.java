package mobility.models;

import jade.util.leap.Serializable; // Changed from java.io.Serializable

/**
 * Represents a location with x and y coordinates
 */
public class Location implements Serializable {
    // No need for serialVersionUID with jade.util.leap.Serializable

    private double x;
    private double y;
    private String name;

    public Location(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    // Rest of the code remains the same
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public String getName() {
        return name;
    }

    /**
     * Calculate distance to another location
     */
    public double distanceTo(Location other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return name + " (" + x + ", " + y + ")";
    }
}