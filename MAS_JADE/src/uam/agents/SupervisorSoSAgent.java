package uam.agents;


import jade.core.Agent;
import uam.behaviors.supervisor.CoordinateJourney;

public class SupervisorSoSAgent extends Agent {

    protected void setup() {
        System.out.println(getLocalName() + ": Waiting for journey requests.");
        addBehaviour(new CoordinateJourney(this));
        // addBehaviour(new CyclicBehaviour() {
        //     public void action() {
        //         ACLMessage msg = receive();
        //         if (msg != null && msg.getConversationId().equals("journey")) {
        //             customerID = msg.getSender().getLocalName();
        //             System.out.println(getLocalName() + ": Received request from " + customerID);

        //             // Send dispatch negotiation to scooter
        //             ACLMessage dispatch = new ACLMessage(ACLMessage.REQUEST);
        //             dispatch.addReceiver(new jade.core.AID("scooter1", jade.core.AID.ISLOCALNAME));
        //             dispatch.setConversationId("dispatch");
        //             dispatch.setContent("DISPATCH_REQUEST");
        //             send(dispatch);
        //         } else if (msg != null && msg.getConversationId().equals("dispatch")) {
        //             if (msg.getContent().equals("ACCEPT")) {
        //                 ACLMessage reply = new ACLMessage(ACLMessage.INFORM);
        //                 reply.addReceiver(new jade.core.AID(customerID, jade.core.AID.ISLOCALNAME));
        //                 reply.setConversationId("journey");
        //                 reply.setContent("PROPOSE_ROUTE:scooter1 ETA=10min");
        //                 send(reply);
        //                 System.out.println(getLocalName() + ": Proposed route to " + customerID);
        //             } else {
        //                 System.out.println(getLocalName() + ": Dispatch rejected.");
        //             }
        //         } else {
        //             block();
        //         }
        //     }
        // });
    }
}