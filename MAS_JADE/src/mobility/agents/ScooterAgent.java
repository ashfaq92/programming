package mobility.agents;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import jade.util.leap.Serializable;

import mobility.models.*;
import java.io.IOException;
import java.util.*;

/**
 * Agent representing an electric scooter
 */
public class ScooterAgent extends Agent {
    // The scooter's current location
    private Location currentLocation;

    // The scooter's current battery level (0-100)
    private int batteryLevel;

    // Whether the scooter is currently assigned to a journey
    private boolean assigned = false;

    // ID of current segment being serviced
    private String currentSegmentId = null;

    @Override
    protected void setup() {
        // Initialize with random location and battery level
        Random random = new Random();
        double x = random.nextDouble() * 10;
        double y = random.nextDouble() * 10;
        currentLocation = new Location(x, y, "ScooterLocation");

        batteryLevel = 50 + random.nextInt(51); // 50-100%

        System.out.println(getLocalName() + ": starting at location " + currentLocation +
                " with battery level " + batteryLevel + "%");

        // Register with DF
        registerWithDF();

        // Add behavior to handle dispatch requests
        addBehaviour(new HandleDispatchRequestBehaviour());

        // Add behavior to handle execution requests
        addBehaviour(new HandleExecuteRequestBehaviour());
    }

    /**
     * Register this agent with the Directory Facilitator
     */
    private void registerWithDF() {
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());

        ServiceDescription sd = new ServiceDescription();
        sd.setType("scooter-resource");
        sd.setName("electric-scooter");

        dfd.addServices(sd);

        try {
            DFService.register(this, dfd);
            System.out.println(getLocalName() + ": registered with DF as scooter resource");
        } catch (FIPAException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void takeDown() {
        try {
            DFService.deregister(this);
        } catch (FIPAException e) {
            e.printStackTrace();
        }
        System.out.println(getLocalName() + ": terminating");
    }

    /**
     * Behavior to handle dispatch negotiation requests
     */
    private class HandleDispatchRequestBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.REQUEST),
                    MessageTemplate.MatchConversationId("negotiate-dispatch"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                try {
                    JourneySegment segment = (JourneySegment) msg.getContentObject();
                    System.out.println(
                            getLocalName() + ": received dispatch request for segment " + segment.getSegmentId());

                    // Make autonomous decision whether to accept the dispatch
                    boolean acceptDispatch = decideOnDispatch(segment);

                    ACLMessage reply = msg.createReply();
                    if (acceptDispatch) {
                        // Accept the dispatch
                        reply.setPerformative(ACLMessage.AGREE);

                        // Update segment with our estimates
                        updateSegmentWithEstimates(segment);

                        assigned = true;
                        currentSegmentId = segment.getSegmentId();

                        try {
                            reply.setContentObject((Serializable) segment);
                            send(reply);
                            System.out.println(getLocalName() + ": ACCEPTED dispatch for segment " +
                                    segment.getSegmentId());
                        } catch (IOException e) {
                            e.printStackTrace();
                            System.out.println(getLocalName() + ": failed to send dispatch acceptance");
                        }
                    } else {
                        // Reject the dispatch
                        reply.setPerformative(ACLMessage.REFUSE);
                        reply.setContent("unavailable");
                        send(reply);
                        System.out.println(getLocalName() + ": REJECTED dispatch for segment " +
                                segment.getSegmentId());
                    }

                } catch (UnreadableException e) {
                    e.printStackTrace();
                    System.out.println(getLocalName() + ": received unreadable dispatch request");
                }
            } else {
                block();
            }
        }

        /**
         * Make a decision on whether to accept a dispatch
         * This is an autonomous behavior
         */
        private boolean decideOnDispatch(JourneySegment segment) {
            // Don't accept if already assigned
            if (assigned) {
                return false;
            }

            // Don't accept if battery too low (< 20%)
            if (batteryLevel < 20) {
                return false;
            }

            // Calculate distance to segment start
            double distanceToStart = currentLocation.distanceTo(segment.getStart());

            // Don't accept if too far away (> 5 units)
            if (distanceToStart > 5) {
                return false;
            }

            // Don't accept if total journey would drain battery
            double journeyDistance = distanceToStart + segment.getStart().distanceTo(segment.getEnd());
            double estimatedBatteryDrain = journeyDistance * 5; // 5% battery per distance unit

            if (batteryLevel - estimatedBatteryDrain < 10) { // Keep 10% reserve
                return false;
            }

            // Otherwise, accept with 90% probability
            return Math.random() < 0.9;
        }

        /**
         * Update segment with our estimates for time and cost
         */
        private void updateSegmentWithEstimates(JourneySegment segment) {
            // Calculate distance
            double distanceToStart = currentLocation.distanceTo(segment.getStart());
            double journeyDistance = segment.getStart().distanceTo(segment.getEnd());
            // double totalDistance = distanceToStart + journeyDistance;

            // Estimate times (30 seconds startup + 1 minute per distance unit)
            long now = System.currentTimeMillis();
            long pickupTime = now + (long) (distanceToStart * 60000) + 30000;
            long arrivalTime = pickupTime + (long) (journeyDistance * 60000);

            // Set estimate time timestamps
            segment.setEstimatedDepartureTime(new Date(pickupTime));
            segment.setEstimatedArrivalTime(new Date(arrivalTime));

            // Estimate cost ($2 base + $1.50 per distance unit)
            double cost = 2.0 + (journeyDistance * 1.5);
            segment.setEstimatedCost(cost);

            // Set vehicle ID
            segment.setVehicleId(getLocalName());
        }
    }

    /**
     * Behavior to handle execution requests
     */
    private class HandleExecuteRequestBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.REQUEST),
                    MessageTemplate.MatchConversationId("execute-journey"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                String segmentId = msg.getContent();
                System.out.println(getLocalName() + ": received execute request for segment " + segmentId);

                // Verify this is our current segment
                if (currentSegmentId != null && currentSegmentId.equals(segmentId)) {
                    // Start execution
                    System.out.println(getLocalName() + ": EXECUTING journey segment " + segmentId);

                    // In a real system, we would update our status periodically
                    // For this example, we'll just acknowledge the execution
                    ACLMessage reply = msg.createReply();
                    reply.setPerformative(ACLMessage.INFORM);
                    reply.setConversationId("journey-status");
                    reply.setContent("executing");
                    send(reply);

                    // Simulate journey completion
                    // In a real system, this would be a separate behavior that runs over time
                    System.out.println(getLocalName() + ": journey segment " + segmentId + " completed");
                    batteryLevel -= 15; // Simulate battery usage
                    System.out.println(getLocalName() + ": battery level now " + batteryLevel + "%");

                    // Reset assignment
                    assigned = false;
                    currentSegmentId = null;
                } else {
                    System.out.println(getLocalName() + ": received execute request for unknown segment " + segmentId);
                }
            } else {
                block();
            }
        }
    }
}