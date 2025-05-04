package uam.behaviors.customer;


import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

import uam.agents.CustomerAgent;

public class ReceiveProposal extends CyclicBehaviour{
    private CustomerAgent customerAgent;

    public ReceiveProposal(CustomerAgent agent) {
        this.customerAgent = agent;
    }

    @Override
    public void action() {
        ACLMessage reply = customerAgent.receive();
        if (reply != null && "journey".equals(reply.getConversationId())) {
            System.out.println(customerAgent.getLocalName() + ": Received proposal -> "
                    + reply.getContent());
            // Optionally, we could send an accept message back here...
            block(); // block until a new message arrives
        } else {
            block();
        }
    }
    
}
