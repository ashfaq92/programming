package mobility.agents;

import jade.core.Agent;
import jade.core.AID;
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
 * Agent representing a supervisor for ground transportation (scooters)
 */
public class ScooterSupervisorAgent extends Agent {
    // Maps segment IDs to segments
    private Map<String, JourneySegment> pendingSegments = new HashMap<>();

    // Maps segment IDs to assigned scooter agents
    private Map<String, AID> segmentAssignments = new HashMap<>();

    // List of available scooter agents
    private List<AID> availableScooters = new ArrayList<>();

    @Override
    protected void setup() {
        System.out.println(getLocalName() + ": starting");

        // Register with DF
        registerWithDF();

        // Add behavior to find scooter agents
        addBehaviour(new FindScootersBehaviour());

        // Add behavior to handle segment planning requests
        addBehaviour(new HandleSegmentRequestBehaviour());

        // Add behavior to handle segment execution requests
        addBehaviour(new HandleExecuteRequestBehaviour());

        // Add behavior to handle scooter responses
        addBehaviour(new HandleScooterResponseBehaviour());
    }

    /**
     * Register this agent with the Directory Facilitator
     */
    private void registerWithDF() {
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());

        ServiceDescription sd = new ServiceDescription();
        sd.setType("ground-supervisor");
        sd.setName("ground-transportation-management");

        dfd.addServices(sd);

        try {
            DFService.register(this, dfd);
            System.out.println(getLocalName() + ": registered with DF as ground supervisor");
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
     * Behavior to find scooter agents
     */
    private class FindScootersBehaviour extends CyclicBehaviour {
        private long lastSearch = 0;
        private static final long SEARCH_INTERVAL = 10000; // 10 seconds

        @Override
        public void action() {
            long now = System.currentTimeMillis();
            if (now - lastSearch > SEARCH_INTERVAL) {
                lastSearch = now;

                DFAgentDescription template = new DFAgentDescription();
                ServiceDescription sd = new ServiceDescription();
                sd.setType("scooter-resource");
                template.addServices(sd);

                try {
                    DFAgentDescription[] results = DFService.search(myAgent, template);
                    availableScooters.clear();
                    for (DFAgentDescription result : results) {
                        AID scooter = result.getName();
                        availableScooters.add(scooter);
                    }
                    System.out.println(getLocalName() + ": found " + availableScooters.size() + " scooter agents");
                } catch (FIPAException e) {
                    e.printStackTrace();
                }
            }
            block(SEARCH_INTERVAL);
        }
    }

    /**
     * Behavior to handle segment planning requests
     */
    private class HandleSegmentRequestBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.REQUEST),
                    MessageTemplate.MatchConversationId("plan-segment"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                try {
                    JourneySegment segment = (JourneySegment) msg.getContentObject();
                    System.out
                            .println(getLocalName() + ": received segment planning request " + segment.getSegmentId());

                    // Store the segment
                    pendingSegments.put(segment.getSegmentId(), segment);

                    // Find an available scooter
                    if (availableScooters.isEmpty()) {
                        System.out.println(
                                getLocalName() + ": no scooters available for segment " + segment.getSegmentId());
                        return;
                    }

                    // For simplicity, use the first available scooter
                    // In a real system, we would select based on proximity, battery level, etc.
                    AID scooter = availableScooters.get(0);

                    // Send dispatch request to scooter
                    ACLMessage dispatchMsg = new ACLMessage(ACLMessage.REQUEST);
                    dispatchMsg.addReceiver(scooter);
                    dispatchMsg.setConversationId("negotiate-dispatch");

                    try {
                        dispatchMsg.setContentObject((Serializable) segment);
                        send(dispatchMsg);
                        System.out.println(getLocalName() + ": sent dispatch request for segment " +
                                segment.getSegmentId() + " to " + scooter.getLocalName());
                    } catch (IOException e) {
                        e.printStackTrace();
                        System.out.println(getLocalName() + ": failed to send dispatch request");
                    }

                } catch (UnreadableException e) {
                    e.printStackTrace();
                    System.out.println(getLocalName() + ": received unreadable segment request");
                }
            } else {
                block();
            }
        }
    }

    /**
     * Behavior to handle segment execution requests
     */
    private class HandleExecuteRequestBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.REQUEST),
                    MessageTemplate.MatchConversationId("execute-segment"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                String segmentId = msg.getContent();
                System.out.println(getLocalName() + ": received execution request for segment " + segmentId);

                // Find the assigned scooter
                AID scooter = segmentAssignments.get(segmentId);
                if (scooter != null) {
                    ACLMessage executeMsg = new ACLMessage(ACLMessage.REQUEST);
                    executeMsg.addReceiver(scooter);
                    executeMsg.setConversationId("execute-journey");
                    executeMsg.setContent(segmentId);
                    send(executeMsg);
                    System.out.println(getLocalName() + ": requested execution of segment " +
                            segmentId + " by " + scooter.getLocalName());
                } else {
                    System.out.println(getLocalName() + ": no scooter assigned to segment " + segmentId);
                }
            } else {
                block();
            }
        }
    }

    /**
     * Behavior to handle scooter responses
     */
    private class HandleScooterResponseBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.or(
                    MessageTemplate.MatchConversationId("dispatch-response"),
                    MessageTemplate.MatchConversationId("journey-status"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                if (msg.getConversationId().equals("dispatch-response")) {
                    handleDispatchResponse(msg);
                } else if (msg.getConversationId().equals("journey-status")) {
                    // Handle journey status updates - not implemented in this example
                    System.out.println(getLocalName() + ": received journey status update from " +
                            msg.getSender().getLocalName());
                }
            } else {
                block();
            }
        }

        private void handleDispatchResponse(ACLMessage msg) {
            try {
                if (msg.getPerformative() == ACLMessage.AGREE) {
                    // Scooter accepted the dispatch
                    JourneySegment segment = (JourneySegment) msg.getContentObject();
                    String segmentId = segment.getSegmentId();
                    AID scooter = msg.getSender();

                    System.out.println(getLocalName() + ": scooter " + scooter.getLocalName() +
                            " accepted dispatch for segment " + segmentId);

                    // Store the assignment
                    segmentAssignments.put(segmentId, scooter);

                    // Update the segment with planning details
                    pendingSegments.put(segmentId, segment);

                    // Notify the supervisor that the segment is planned
                    ACLMessage notifyMsg = new ACLMessage(ACLMessage.INFORM);
                    notifyMsg.addReceiver(new AID("supervisor", AID.ISLOCALNAME));
                    notifyMsg.setConversationId("segment-planned");

                    try {
                        notifyMsg.setContentObject((Serializable) segment);
                        send(notifyMsg);
                        System.out.println(getLocalName() + ": notified supervisor that segment " +
                                segmentId + " is planned");
                    } catch (IOException e) {
                        e.printStackTrace();
                        System.out.println(getLocalName() + ": failed to send segment planned notification");
                    }
                } else {
                    // Scooter rejected the dispatch
                    System.out.println(getLocalName() + ": scooter " + msg.getSender().getLocalName() +
                            " rejected dispatch");

                    // In a real system, we would try another scooter
                }
            } catch (UnreadableException e) {
                e.printStackTrace();
                System.out.println(getLocalName() + ": received unreadable dispatch response");
            }
        }
    }
}