package bookstore.behaviors.manager;

import jade.core.behaviours.WakerBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.ManagerAgent;

/**
 * Behavior for updating book inventory
 */
public class UpdateInventoryBehavior extends WakerBehaviour {
    private ManagerAgent managerAgent;

    /**
     * Constructor
     * 
     * @param agent      The manager agent
     * @param wakeupTime Time to wait before executing the behavior (in
     *                   milliseconds)
     */
    public UpdateInventoryBehavior(ManagerAgent agent, long wakeupTime) {
        super(agent, wakeupTime);
        this.managerAgent = agent;
    }

    /**
     * Method called when the wakeup time is reached
     */
    @Override
    protected void onWake() {
        // Check if we have a bookstore agent to talk to
        if (managerAgent.getBookstoreAgent() == null) {
            System.out.println("Manager can't update inventory: no bookstore found!");
            return;
        }

        // Create an update message
        ACLMessage updateMsg = new ACLMessage(ACLMessage.PROPOSE);
        updateMsg.addReceiver(managerAgent.getBookstoreAgent());

        // Let's add more copies of "The Great Gatsby" (assuming we know its ISBN)
        // In a real system, we'd have a UI for the manager to specify what to update
        updateMsg.setContent("978-0743273565:10");

        System.out.println("Manager is updating inventory: 978-0743273565 to 10 copies");

        // Send the message
        managerAgent.send(updateMsg);

        // Wait for a reply
        MessageTemplate mt = MessageTemplate.or(
                MessageTemplate.MatchPerformative(ACLMessage.AGREE),
                MessageTemplate.MatchPerformative(ACLMessage.REFUSE));

        ACLMessage reply = managerAgent.blockingReceive(mt, 5000); // Wait up to 5 seconds

        if (reply != null) {
            if (reply.getPerformative() == ACLMessage.AGREE) {
                // Update successful
                System.out.println("Manager updated inventory successfully!");
            } else {
                // Update failed
                System.out.println("Manager couldn't update inventory: " + reply.getContent());
            }
        } else {
            // No reply received (timeout)
            System.out.println("Manager received no update reply from bookstore!");
        }
    }
}