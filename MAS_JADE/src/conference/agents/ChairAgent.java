package conference.agents;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;


public class ChairAgent extends Agent {
    protected void setup() {
        System.out.println(getLocalName() + ": read to classify submissions");
        addBehaviour(new CyclicBehaviour() {
            public void action() {
                ACLMessage msg = receive();
                if (msg != null && msg.getConversationId().equals("submit-paper")) {
                    String content = msg.getContent();
                    // simple accept / reject logic
                    String decision = content.contains("Gaia") ? "accept" : "reject";
                    ACLMessage reply = msg.createReply();
                    reply.setPerformative(ACLMessage.INFORM);
                    reply.setContent("P1" + decision);
                    send(reply);
                    System.out.println(getLocalName() + ": replied "+ decision);
                } else {
                    block();
                }
            }
        });
    }
    protected void takeDown() {
        System.out.println(getBehavioursCnt() + ": terminating.");
    }    
}
