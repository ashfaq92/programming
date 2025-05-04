package bookstore.agents;

import jade.core.Agent;
import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;

import bookstore.behaviors.customer.QueryBookBehavior;
import bookstore.behaviors.customer.PurchaseBookBehavior;

/**
 * Agent representing a customer who wants to buy books
 */
public class CustomerAgent extends Agent {
    // The title of the book the customer wants to buy
    private String targetBookTitle;

    // The ISBN of the book once found
    private String targetBookISBN;

    // Reference to the bookstore agent
    private AID bookstoreAgent;

    /**
     * Setup method called when the agent is initialized
     */
    @Override
    protected void setup() {
        // Get the title of the book to buy as a start-up argument
        Object[] args = getArguments();
        if (args != null && args.length > 0) {
            targetBookTitle = (String) args[0];
            System.out.println("Customer agent " + getLocalName() + " is ready. Looking for: " + targetBookTitle);

            // Search for the bookstore agent
            findBookstore();

            // Add a one-shot behavior to query for the book
            addBehaviour(new QueryBookBehavior(this, targetBookTitle));
        } else {
            System.out.println("Customer agent " + getLocalName() + " doesn't know what book to look for!");
            doDelete(); // Terminate the agent if no book title is provided
        }
    }

    /**
     * Method to find the bookstore agent in the Directory Facilitator
     */
    private void findBookstore() {
        // Create a template for searching bookstore agents
        DFAgentDescription template = new DFAgentDescription();
        ServiceDescription sd = new ServiceDescription();
        sd.setType("book-query"); // We're looking for agents that can respond to book queries
        template.addServices(sd);

        try {
            // Search the DF for matching agents
            DFAgentDescription[] result = DFService.search(this, template);
            if (result.length > 0) {
                bookstoreAgent = result[0].getName();
                System.out.println("Customer found bookstore agent: " + bookstoreAgent.getLocalName());
            } else {
                System.out.println("Customer couldn't find a bookstore agent!");
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
        System.out.println("Customer agent " + getLocalName() + " terminating.");
    }

    /**
     * Get the bookstore agent
     * 
     * @return The AID of the bookstore agent
     */
    public AID getBookstoreAgent() {
        return bookstoreAgent;
    }

    /**
     * Set the target book ISBN once found
     * 
     * @param isbn The ISBN of the found book
     */
    public void setTargetBookISBN(String isbn) {
        this.targetBookISBN = isbn;
        // Now that we have the ISBN, initiate the purchase
        addBehaviour(new PurchaseBookBehavior(this, isbn));
    }

    /**
     * Get the target book ISBN
     * 
     * @return The ISBN of the target book
     */
    public String getTargetBookISBN() {
        return targetBookISBN;
    }

    /**
     * Get the target book title
     * 
     * @return The title of the target book
     */
    public String getTargetBookTitle() {
        return targetBookTitle;
    }
}