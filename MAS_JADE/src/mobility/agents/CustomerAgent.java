package mobility.agents;

import jade.core.Agent;
import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
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
import java.util.Date;
import java.util.Calendar;

/**
 * Agent representing a customer who requests journeys
 */
public class CustomerAgent extends Agent {
    private AID supervisorAgent;
    private JourneyRequest currentRequest;
    private Location customerLocation;
    private Location destinationLocation;

    @Override
    protected void setup() {
        // Initialize customer location and destination
        Object[] args = getArguments();
        if (args != null && args.length >= 4) {
            double startX = Double.parseDouble(args[0].toString());
            double startY = Double.parseDouble(args[1].toString());
            double destX = Double.parseDouble(args[2].toString());
            double destY = Double.parseDouble(args[3].toString());

            customerLocation = new Location(startX, startY, "CustomerLocation");
            destinationLocation = new Location(destX, destY, "DestinationLocation");

            System.out.println(getLocalName() + ": initialized with location " + customerLocation +
                    " and destination " + destinationLocation);
        } else {
            // Default locations if not provided
            customerLocation = new Location(0, 0, "CustomerLocation");
            destinationLocation = new Location(10, 10, "DestinationLocation");
            System.out.println(getLocalName() + ": using default locations");
        }

        // Register with DF
        registerWithDF();

        // Find the SupervisorSoS agent
        findSupervisor();

        // Add behavior to request a journey
        addBehaviour(new RequestJourneyBehaviour());

        // Add behavior to handle responses
        addBehaviour(new HandleProposalBehaviour());
    }

    /**
     * Register this agent with the Directory Facilitator
     */
    private void registerWithDF() {
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());

        ServiceDescription sd = new ServiceDescription();
        sd.setType("journey-requester");
        sd.setName("journey-requesting");

        dfd.addServices(sd);

        try {
            DFService.register(this, dfd);
            System.out.println(getLocalName() + ": registered with DF as journey requester");
        } catch (FIPAException e) {
            e.printStackTrace();
        }
    }

    /**
     * Find the SupervisorSoS agent through the DF
     */
    private void findSupervisor() {
        DFAgentDescription template = new DFAgentDescription();
        ServiceDescription sd = new ServiceDescription();
        sd.setType("journey-coordinator");
        template.addServices(sd);

        try {
            DFAgentDescription[] results = DFService.search(this, template);
            if (results.length > 0) {
                supervisorAgent = results[0].getName();
                System.out.println(getLocalName() + ": found supervisor agent " + supervisorAgent.getLocalName());
            } else {
                System.out.println(getLocalName() + ": could not find a supervisor agent");
            }
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
     * Behavior to request a journey
     */
    private class RequestJourneyBehaviour extends OneShotBehaviour {
        @Override
        public void action() {
            if (supervisorAgent == null) {
                System.out.println(getLocalName() + ": cannot request journey, no supervisor found");
                return;
            }

            // Create a journey request
            String requestId = "REQ-" + System.currentTimeMillis();

            // Set deadline 30 minutes from now
            Calendar calendar = Calendar.getInstance();
            calendar.add(Calendar.MINUTE, 30);
            Date deadline = calendar.getTime();

            currentRequest = new JourneyRequest(
                    requestId,
                    getLocalName(),
                    customerLocation,
                    destinationLocation,
                    deadline);

            // Random priority (20% chance of being priority)
            if (Math.random() < 0.2) {
                currentRequest.setPriority(true);
                System.out.println(getLocalName() + ": setting priority request");
            }

            // Create and send the message
            ACLMessage requestMsg = new ACLMessage(ACLMessage.REQUEST);
            requestMsg.addReceiver(supervisorAgent);
            requestMsg.setConversationId("request-journey");

            try {
                requestMsg.setContentObject((Serializable) currentRequest);
                send(requestMsg);
                System.out.println(getLocalName() + ": sent journey request " + requestId +
                        " from " + customerLocation + " to " + destinationLocation);
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println(getLocalName() + ": failed to send journey request");
            }
        }
    }

    /**
     * Behavior to handle route proposals
     */
    private class HandleProposalBehaviour extends CyclicBehaviour {
        @Override
        public void action() {
            MessageTemplate mt = MessageTemplate.and(
                    MessageTemplate.MatchPerformative(ACLMessage.PROPOSE),
                    MessageTemplate.MatchConversationId("propose-route"));

            ACLMessage msg = myAgent.receive(mt);
            if (msg != null) {
                // Proposal received
                try {
                    RouteProposal proposal = (RouteProposal) msg.getContentObject();
                    System.out.println(getLocalName() + ": received route proposal " + proposal.getProposalId());
                    System.out.println(getLocalName() + ": route details: " + proposal);

                    // Make decision to accept or reject (autonomous behavior)
                    boolean acceptProposal = decideOnProposal(proposal);

                    // Send response
                    ACLMessage reply = msg.createReply();
                    if (acceptProposal) {
                        reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
                        System.out.println(getLocalName() + ": ACCEPTING route proposal");
                    } else {
                        reply.setPerformative(ACLMessage.REJECT_PROPOSAL);
                        System.out.println(getLocalName() + ": REJECTING route proposal");
                    }
                    reply.setContent(proposal.getProposalId());
                    send(reply);

                } catch (UnreadableException e) {
                    e.printStackTrace();
                    System.out.println(getLocalName() + ": received unreadable proposal");
                }
            } else {
                block();
            }
        }

        /**
         * Make a decision on whether to accept a proposal
         * This is an autonomous behavior
         */
        private boolean decideOnProposal(RouteProposal proposal) {
            // Simple logic: accept if cost is under 50 or if we marked it as priority
            if (proposal.getTotalEstimatedCost() < 50 ||
                    (currentRequest != null && currentRequest.isPriority())) {
                return true;
            }

            // Otherwise, 70% chance of accepting
            return Math.random() < 0.7;
        }
    }
}