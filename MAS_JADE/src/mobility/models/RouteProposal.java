package mobility.models;

import jade.util.leap.Serializable; // Changed from java.io.Serializable
import jade.util.leap.ArrayList; // Use JADE's ArrayList
import jade.util.leap.List; // Use JADE's List
import java.util.Date;

/**
 * Represents a complete route proposal for a journey request
 */
public class RouteProposal implements Serializable {
    // No need for serialVersionUID with jade.util.leap.Serializable

    private String proposalId;
    private String requestId;
    private List segments; // Changed from java.util.List to jade.util.leap.List
    private Date totalEstimatedTime;
    private double totalEstimatedCost;

    public RouteProposal(String proposalId, String requestId) {
        this.proposalId = proposalId;
        this.requestId = requestId;
        this.segments = new ArrayList(); // Changed from java.util.ArrayList to jade.util.leap.ArrayList
    }

    // Getters and setters adapted to JADE collections
    public String getProposalId() {
        return proposalId;
    }

    public String getRequestId() {
        return requestId;
    }

    public List getSegments() {
        return segments;
    }

    public void addSegment(JourneySegment segment) {
        segments.add(segment);
    }

    public Date getTotalEstimatedTime() {
        return totalEstimatedTime;
    }

    public void setTotalEstimatedTime(Date totalEstimatedTime) {
        this.totalEstimatedTime = totalEstimatedTime;
    }

    public double getTotalEstimatedCost() {
        return totalEstimatedCost;
    }

    public void setTotalEstimatedCost(double totalEstimatedCost) {
        this.totalEstimatedCost = totalEstimatedCost;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("RouteProposal{proposalId='").append(proposalId)
                .append("', segments=[");

        for (int i = 0; i < segments.size(); i++) {
            if (i > 0) {
                sb.append(" -> ");
            }
            JourneySegment segment = (JourneySegment) segments.get(i); // Need explicit cast with JADE collections
            sb.append(segment.getMode()).append(": ")
                    .append(segment.getStart().getName())
                    .append(" to ")
                    .append(segment.getEnd().getName());
        }

        sb.append("], totalEstimatedCost=").append(totalEstimatedCost)
                .append(", totalEstimatedTime=").append(totalEstimatedTime)
                .append('}');

        return sb.toString();
    }
}