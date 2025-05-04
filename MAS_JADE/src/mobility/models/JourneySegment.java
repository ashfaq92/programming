package mobility.models;

import jade.util.leap.Serializable; // Changed from java.io.Serializable
import java.util.Date;

/**
 * Represents a segment of a journey with a specific mode of transport
 */
public class JourneySegment implements Serializable {
    // No need for serialVersionUID with jade.util.leap.Serializable

    public enum TransportMode {
        GROUND, AIR
    }

    private String segmentId;
    private String requestId;
    private TransportMode mode;
    private Location start;
    private Location end;
    private Date estimatedDepartureTime;
    private Date estimatedArrivalTime;
    private String vehicleId;
    private double estimatedCost;

    // Rest of the code remains the same
    public JourneySegment(String segmentId, String requestId, TransportMode mode,
            Location start, Location end) {
        this.segmentId = segmentId;
        this.requestId = requestId;
        this.mode = mode;
        this.start = start;
        this.end = end;
    }

    // All getters and setters remain the same
    public String getSegmentId() {
        return segmentId;
    }

    public String getRequestId() {
        return requestId;
    }

    public TransportMode getMode() {
        return mode;
    }

    public Location getStart() {
        return start;
    }

    public Location getEnd() {
        return end;
    }

    public Date getEstimatedDepartureTime() {
        return estimatedDepartureTime;
    }

    public void setEstimatedDepartureTime(Date estimatedDepartureTime) {
        this.estimatedDepartureTime = estimatedDepartureTime;
    }

    public Date getEstimatedArrivalTime() {
        return estimatedArrivalTime;
    }

    public void setEstimatedArrivalTime(Date estimatedArrivalTime) {
        this.estimatedArrivalTime = estimatedArrivalTime;
    }

    public String getVehicleId() {
        return vehicleId;
    }

    public void setVehicleId(String vehicleId) {
        this.vehicleId = vehicleId;
    }

    public double getEstimatedCost() {
        return estimatedCost;
    }

    public void setEstimatedCost(double estimatedCost) {
        this.estimatedCost = estimatedCost;
    }

    @Override
    public String toString() {
        return "JourneySegment{" +
                "segmentId='" + segmentId + '\'' +
                ", mode=" + mode +
                ", start=" + start +
                ", end=" + end +
                ", estimatedDepartureTime=" + estimatedDepartureTime +
                ", estimatedArrivalTime=" + estimatedArrivalTime +
                ", vehicleId='" + vehicleId + '\'' +
                ", estimatedCost=" + estimatedCost +
                '}';
    }
}