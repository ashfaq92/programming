package bookstore.agents;

import jade.core.Agent;
import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;

import bookstore.behaviors.manager.UpdateInventoryBehavior;

/**
 * Agent representing a store manager who can update inventory
 */
public class ManagerAgent extends Agent {
    // Reference to the bookstore agent
    private AID bookstoreAgent;

    /**
     * Setup method called when the agent is initialized
     */
    @Override
    protected void setup() {
        System.out.println("Manager agent " + getLocalName() + " is ready.");

        // Search for the bookstore agent
        findBookstore();

        // Schedule an inventory update after 5 seconds
        addBehaviour(new UpdateInventoryBehavior(this, 5000));
    }

    /**
     * Method to find the bookstore agent in the Directory Facilitator
     */
    private void findBookstore() {
        // Create a template for searching bookstore agents
        DFAgentDescription template = new DFAgentDescription();
        ServiceDescription sd = new ServiceDescription();
        sd.setType("inventory-management"); // We're looking for agents that can handle inventory updates
        template.addServices(sd);

        try {
            // Search the DF for matching agents
            DFAgentDescription[] result = DFService.search(this, template);
            if (result.length > 0) {
                bookstoreAgent = result[0].getName();
                System.out.println("Manager found bookstore agent: " + bookstoreAgent.getLocalName());
            } else {
                System.out.println("Manager couldn't find a bookstore agent!");
            }
        } catch (FIPAException e) {
            e.printStackTrace();
        }
    }

    /**
     * Called when the agent is terminated
     */
    @Override
    protected void takeDown() {
        System.out.println("Manager agent " + getLocalName() + " terminating.");
    }

    /**
     * Get the bookstore agent
     * 
     * @return The AID of the bookstore agent
     */
    public AID getBookstoreAgent() {
        return bookstoreAgent;
    }
}