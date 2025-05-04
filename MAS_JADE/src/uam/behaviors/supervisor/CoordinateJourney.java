package uam.behaviors.supervisor;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import uam.agents.SupervisorSoSAgent;


/**
 * Handles the end-to-end journey coordination:
 * 1. Receives REQUEST_JOURNEY from Customer → sends DISPATCH_REQUEST to Scooter
 * 2. Receives ACCEPT/REJECT from Scooter → sends PROPOSE_ROUTE to Customer
 * 3. (Later) Handles ACCEPT_PROPOSAL from Customer
 */
public class CoordinateJourney extends CyclicBehaviour {
    private SupervisorSoSAgent supervisorSoSAgent;
    private String customerID = null;

    public CoordinateJourney(SupervisorSoSAgent agent) {
        this.supervisorSoSAgent = agent;
    }

    
    @Override
    public void action() {
        ACLMessage msg = supervisorSoSAgent.receive();
        if (msg == null) {
            block();
            return;
        }

        String conv = msg.getConversationId();
        switch (conv) {
            case "journey":
                customerID = msg.getSender().getLocalName();
                System.out.println(getAgent().getLocalName() +
                        ": Received journey request from " + customerID);
                ACLMessage dispatch = new ACLMessage(ACLMessage.REQUEST);
                dispatch.addReceiver(new AID("scooter1", AID.ISLOCALNAME));
                dispatch.setConversationId("dispatch");
                dispatch.setContent("DISPATCH_REQUEST");
                getAgent().send(dispatch);
                break;

            case "dispatch":
                if ("ACCEPT".equals(msg.getContent())) {
                    ACLMessage propose = new ACLMessage(ACLMessage.INFORM);
                    propose.addReceiver(new AID(customerID, AID.ISLOCALNAME));
                    propose.setConversationId("journey");
                    propose.setContent("PROPOSE_ROUTE:scooter1 ETA=10min");
                    getAgent().send(propose);
                    System.out.println(getAgent().getLocalName() +
                            ": Proposed route to " + customerID);
                } else {
                    System.out.println(getAgent().getLocalName() +
                            ": Dispatch rejected by scooter");
                }
                break;

            case "accept-proposal":
                System.out.println(getAgent().getLocalName() +
                        ": Customer accepted the proposal");
                // (Future extension: trigger execution here)
                break;

            default:
                block();
        }
    }
}
