package conference.agents;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;



public class AuthorAgent extends Agent {
    protected void setup() {
        System.out.println(getLocalName() + ": ready to submit paper.");
        addBehaviour(new OneShotBehaviour() {
            public void action() {
                ACLMessage msg = new ACLMessage(ACLMessage.REQUEST);
                msg.addReceiver(new jade.core.AID("chair", jade.core.AID.ISLOCALNAME));
                msg.setConversationId("submit-paper");
                msg.setContent("P1: Refinement of Gaia Design");
                send(msg);
                System.out.println(getLocalName() + ": submitted P1");
            }
        });
    }

    protected void takeDown() {
        System.out.println(getLocalName() + ": terminating.");
    }
}