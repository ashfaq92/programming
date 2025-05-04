package mobility.agents;

import jade.core.Agent;
import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
// import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.WakerBehaviour;
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
 * Agent representing the System of Systems Supervisor that coordinates journeys
 */
public class SupervisorSoSAgent extends Agent {
    // Maps request IDs to their journey requests
    private Map<String, JourneyRequest> pendingRequests = new HashMap<>();

    // Maps request IDs to their segments
    private Map<String, List<JourneySegment>> requestSegments = new HashMap<>();

    // Maps segment IDs to the agent responsible for that segment
    private Map<String, AID> segmentResponsibilities = new HashMap<>();

    // A map of transport supervisor agents by mode
    private Map<JourneySegment.TransportMode, List<AID>> transportSupervisors = new HashMap<>();

    @Override
    protected void setup() {
        System.out.println(getLocalName() + ": starting");

        // Initialize transport supervisors map
        for (JourneySegment.TransportMode mode : JourneySegment.TransportMode.values()) {
            transportSupervisors.put(mode, new ArrayList<>());
        }

        // Register with DF
        registerWithDF();

        // Add behavior to find transport supervisors
        addBehaviour(new FindTransportSupervisorsBehaviour());

        // Add behavior to handle journey requests
        addBehaviour(new HandleJourneyRequestBehaviour());

        // Add behavior to handle segment responses
        addBehaviour(new HandleSegmentResponseBehaviour());

        // Add behavior to handle route acceptance/rejection
        addBehaviour(new HandleRouteResponseBehaviour());
    }

    /**
     * Register this agent with the Directory Facilitator
     */
    private void registerWithDF() {
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());

        ServiceDescription sd = new ServiceDescription();
        sd.setType("journey-coordinator");
        sd.setName("journey-coordination");

        dfd.addServices(sd);

        try {
            DFService.register(this, dfd);
            System.out.println(getLocalName() + ": registered with DF as journey coordinator");
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
     * Behavior to find transport supervisors
     */
    private class FindTransportSupervisorsBehaviour extends CyclicBehaviour {
        private long lastSearch = 0;
        private static final long SEARCH_INTERVAL = 5000; // 5 seconds

        @Override
        public void action() {
            long now = System.currentTimeMillis();
            if (now - lastSearch > SEARCH_INTERVAL) {
                lastSearch = now;

                // Find ground supervisors
                findSupervisors(JourneySegment.TransportMode.GROUND, "ground-supervisor");

                // Find air supervisors
                findSupervisors(JourneySegment.TransportMode.AIR, "air-supervisor");

                System.out.println(getLocalName() + ": found " +
                        transportSupervisors.get(JourneySegment.TransportMode.GROUND).size() +
                        " ground supervisors and " +
                        transportSupervisors.get(JourneySegment.TransportMode.AIR).size() +
                        " air supervisors");
            }
            block(1000); // Block for 1 second
        }

        private void findSupervisors(JourneySegment.TransportMode mode, String serviceType) {
            System.out.println(getLocalName() + ": searching for " + serviceType + " services");

            DFAgentDescription template = new DFAgentDescription();
            ServiceDescription sd = new ServiceDescription();
            sd.setType(serviceType);
            template.addServices(sd);

            try {
                DFAgentDescription[] results = DFService.search(myAgent, template);
                System.out.println(getLocalName() + ": found " + results.length + " results for " + serviceType);

                // Clear previous supervisors of this mode
                transportSupervisors.get(mode).clear();

                for (DFAgentDescription result : results) {
                    AID supervisor = result.getName();
                    if (!transportSupervisors.get(mode).contains(supervisor)) {
                        transportSupervisors.get(mode).add(supervisor);
                        System.out.println(
                                getLocalName() + ": found " + mode + " supervisor " + supervisor.getLocalName());
                    }
                }
            } catch (FIPAException e) {
                e.printStackTrace();
                System.out.println(getLocalName() + ": error searching for " + serviceType + ": " + e.getMessage());
            }
        }
    }

    /**
     * Behavior to handle journey requests from customers
     */
    private class HandleJourneyRequestBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.REQUEST),
                    MessageTemplate.MatchConversationId("request-journey"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                try {
                    JourneyRequest request = (JourneyRequest) msg.getContentObject();
                    System.out.println(getLocalName() + ": received journey request " + request.getRequestId() +
                            " from " + msg.getSender().getLocalName());

                    // Store the request
                    pendingRequests.put(request.getRequestId(), request);
                    requestSegments.put(request.getRequestId(), new ArrayList<>());

                    // Create and send journey segments
                    createJourneySegments(request, msg.getSender());

                } catch (UnreadableException e) {
                    e.printStackTrace();
                    System.out.println(getLocalName() + ": received unreadable journey request");
                }
            } else {
                block();
            }
        }

        /**
         * Create journey segments for a request and send them to appropriate
         * supervisors
         */
        private void createJourneySegments(JourneyRequest request, AID customer) {
            // For this example, we'll create a single ground segment
            String segmentId = "SEG-" + System.currentTimeMillis();
            JourneySegment segment = new JourneySegment(
                    segmentId,
                    request.getRequestId(),
                    JourneySegment.TransportMode.GROUND,
                    request.getStart(),
                    request.getDestination());

            // Store the segment
            requestSegments.get(request.getRequestId()).add(segment);

            // Find an appropriate supervisor
            List<AID> supervisors = transportSupervisors.get(JourneySegment.TransportMode.GROUND);
            if (supervisors.isEmpty()) {
                System.out.println(getLocalName() + ": no ground supervisors available yet, will retry later");

                // Schedule a retry in 5 seconds
                myAgent.addBehaviour(new WakerBehaviour(myAgent, 5000) {
                    @Override
                    protected void onWake() {
                        createJourneySegments(request, customer);
                    }
                });
                return;
            }

            // For simplicity, use the first supervisor
            AID supervisor = supervisors.get(0);
            segmentResponsibilities.put(segmentId, supervisor);

            // Send the segment request
            ACLMessage segmentMsg = new ACLMessage(ACLMessage.REQUEST);
            segmentMsg.addReceiver(supervisor);
            segmentMsg.setConversationId("plan-segment");

            try {
                segmentMsg.setContentObject((Serializable) segment);
                send(segmentMsg);
                System.out.println(getLocalName() + ": sent segment " + segmentId +
                        " to " + supervisor.getLocalName());
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println(getLocalName() + ": failed to send segment request");
            }
        }
    }

    /**
     * Behavior to handle responses for journey segments
     */
    private class HandleSegmentResponseBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.INFORM),
                    MessageTemplate.MatchConversationId("segment-planned"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                try {
                    JourneySegment segment = (JourneySegment) msg.getContentObject();
                    System.out.println(getLocalName() + ": received planned segment " + segment.getSegmentId() +
                            " from " + msg.getSender().getLocalName());

                    // Update our segment with planning details
                    updateSegment(segment);

                    // Check if all segments for this request are planned
                    String requestId = segment.getRequestId();
                    List<JourneySegment> segments = requestSegments.get(requestId);

                    boolean allPlanned = true;
                    for (JourneySegment s : segments) {
                        if (s.getVehicleId() == null) { // If no vehicle assigned, not planned yet
                            allPlanned = false;
                            break;
                        }
                    }

                    if (allPlanned) {
                        // All segments planned, send route proposal to customer
                        sendRouteProposal(requestId);
                    }

                } catch (UnreadableException e) {
                    e.printStackTrace();
                    System.out.println(getLocalName() + ": received unreadable segment response");
                }
            } else {
                block();
            }
        }

        /**
         * Update a segment with planning details
         */
        private void updateSegment(JourneySegment updatedSegment) {
            String requestId = updatedSegment.getRequestId();
            List<JourneySegment> segments = requestSegments.get(requestId);

            for (int i = 0; i < segments.size(); i++) {
                if (segments.get(i).getSegmentId().equals(updatedSegment.getSegmentId())) {
                    segments.set(i, updatedSegment);
                    break;
                }
            }
        }

        /**
         * Send a route proposal to the customer
         */
        private void sendRouteProposal(String requestId) {
            JourneyRequest request = pendingRequests.get(requestId);
            List<JourneySegment> segments = requestSegments.get(requestId);

            // Create a route proposal
            String proposalId = "PROP-" + System.currentTimeMillis();
            RouteProposal proposal = new RouteProposal(proposalId, requestId);

            // Add segments to proposal
            double totalCost = 0;
            Date latestArrival = null;

            for (JourneySegment segment : segments) {
                proposal.addSegment(segment);
                totalCost += segment.getEstimatedCost();

                if (latestArrival == null || segment.getEstimatedArrivalTime().after(latestArrival)) {
                    latestArrival = segment.getEstimatedArrivalTime();
                }
            }

            proposal.setTotalEstimatedCost(totalCost);
            proposal.setTotalEstimatedTime(latestArrival);

            // Apply priority discount if applicable (autonomous decision)
            if (request.isPriority()) {
                // 15% discount for priority customers
                double discountedCost = totalCost * 0.85;
                proposal.setTotalEstimatedCost(discountedCost);
                System.out.println(getLocalName() + ": applied priority discount to proposal " + proposalId);
            }

            // Send the proposal to the customer
            ACLMessage proposalMsg = new ACLMessage(ACLMessage.PROPOSE);
            proposalMsg.addReceiver(new AID(request.getUserId(), AID.ISLOCALNAME));
            proposalMsg.setConversationId("propose-route");

            try {
                proposalMsg.setContentObject((Serializable) proposal);
                send(proposalMsg);
                System.out.println(getLocalName() + ": sent route proposal " + proposalId +
                        " to " + request.getUserId());
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println(getLocalName() + ": failed to send route proposal");
            }
        }
    }

    /**
     * Behavior to handle customer responses to route proposals
     */
    private class HandleRouteResponseBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.or(
                    MessageTemplate.MatchPerformative(ACLMessage.ACCEPT_PROPOSAL),
                    MessageTemplate.MatchPerformative(ACLMessage.REJECT_PROPOSAL));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                String proposalId = msg.getContent();
                boolean accepted = (msg.getPerformative() == ACLMessage.ACCEPT_PROPOSAL);

                System.out.println(getLocalName() + ": route proposal " + proposalId +
                        " was " + (accepted ? "ACCEPTED" : "REJECTED") +
                        " by " + msg.getSender().getLocalName());

                if (accepted) {
                    // Find the request ID from the proposal ID
                    // In a real system, we would store proposals with their request IDs
                    // For simplicity, we'll assume the format PROP-{timestamp}

                    // Notify vehicle supervisors to execute the journey
                    for (Map.Entry<String, List<JourneySegment>> entry : requestSegments.entrySet()) {
                        for (JourneySegment segment : entry.getValue()) {
                            AID supervisor = segmentResponsibilities.get(segment.getSegmentId());
                            if (supervisor != null) {
                                ACLMessage executeMsg = new ACLMessage(ACLMessage.REQUEST);
                                executeMsg.addReceiver(supervisor);
                                executeMsg.setConversationId("execute-segment");
                                executeMsg.setContent(segment.getSegmentId());
                                send(executeMsg);
                                System.out.println(getLocalName() + ": requested execution of segment " +
                                        segment.getSegmentId() + " by " + supervisor.getLocalName());
                            }
                        }
                    }
                }
            } else {
                block();
            }
        }
    }
}