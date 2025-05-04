package uam.behaviors.customer;

import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;

import uam.agents.CustomerAgent;

public class RequestJourney extends OneShotBehaviour {
    private CustomerAgent customerAgent;
    
    public RequestJourney(CustomerAgent agent) {
        this.customerAgent = agent;
    }

    @Override
    public void action() {
        ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);
        msg.addReceiver(new jade.core.AID("supervisor", jade.core.AID.ISLOCALNAME));
        msg.setConversationId("journey");
        msg.setContent("REQUEST_JOURNEY:X,Y,30min");
        customerAgent.send(msg);
        System.out.println(customerAgent.getLocalName() + ": Sent journey request.");
    }
}
