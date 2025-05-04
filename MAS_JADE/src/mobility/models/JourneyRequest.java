package mobility.models;

import jade.util.leap.Serializable; // Changed from java.io.Serializable
import java.util.Date;

/**
 * Represents a journey request from a customer
 */
public class JourneyRequest implements Serializable {
    // No need for serialVersionUID with jade.util.leap.Serializable

    private String requestId;
    private String userId;
    private Location start;
    private Location destination;
    private Date deadline;
    private boolean isPriority;

    // Rest of the code remains the same
    public JourneyRequest(String requestId, String userId, Location start, Location destination, Date deadline) {
        this.requestId = requestId;
        this.userId = userId;
        this.start = start;
        this.destination = destination;
        this.deadline = deadline;
        this.isPriority = false;
    }

    // Getters and setters remain the same
    public String getRequestId() {
        return requestId;
    }

    public String getUserId() {
        return userId;
    }

    public Location getStart() {
        return start;
    }

    public Location getDestination() {
        return destination;
    }

    public Date getDeadline() {
        return deadline;
    }

    public boolean isPriority() {
        return isPriority;
    }

    public void setPriority(boolean priority) {
        isPriority = priority;
    }

    @Override
    public String toString() {
        return "JourneyRequest{" +
                "requestId='" + requestId + '\'' +
                ", userId='" + userId + '\'' +
                ", start=" + start +
                ", destination=" + destination +
                ", deadline=" + deadline +
                ", isPriority=" + isPriority +
                '}';
    }
}