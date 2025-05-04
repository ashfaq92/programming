package uam.behaviors.scooter;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import uam.agents.ScooterAgent;

public class HandleDispatchRequest extends CyclicBehaviour {
    private ScooterAgent scooterAgent;
    
    public HandleDispatchRequest(ScooterAgent agent) {
        this.scooterAgent = agent;
    }

    @Override
    public void action() {
        ACLMessage msg = scooterAgent.receive();
        if (msg != null && msg.getConversationId().equals("dispatch")) {
            ACLMessage reply = msg.createReply();
            if (!scooterAgent.busy) {
                scooterAgent.busy = true;
                reply.setPerformative(ACLMessage.AGREE);
                reply.setContent("ACCEPT");
                System.out.println(scooterAgent.getLocalName() + ": Accepted dispatch.");
            } else {
                reply.setPerformative(ACLMessage.REFUSE);
                reply.setContent("REJECT");
                System.out.println(scooterAgent.getLocalName() + ": Rejected dispatch (busy).");
            }
            scooterAgent.send(reply);
        } else {
            block();
        }
    }
}
