package bookstore.behaviors.customer;

import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.CustomerAgent;

/**
 * Behavior for purchasing a book by ISBN
 */
public class PurchaseBookBehavior extends OneShotBehaviour {
    private CustomerAgent customerAgent;
    private String bookISBN;

    /**
     * Constructor
     * 
     * @param agent The customer agent
     * @param isbn  The ISBN of the book to purchase
     */
    public PurchaseBookBehavior(CustomerAgent agent, String isbn) {
        this.customerAgent = agent;
        this.bookISBN = isbn;
    }

    /**
     * Method called when the behavior is executed
     */
    @Override
    public void action() {
        // Check if we have a bookstore agent to talk to
        if (customerAgent.getBookstoreAgent() == null) {
            System.out.println("Customer can't purchase book: no bookstore found!");
            return;
        }

        // Create a purchase request message
        ACLMessage purchaseMsg = new ACLMessage(ACLMessage.REQUEST);
        purchaseMsg.addReceiver(customerAgent.getBookstoreAgent());
        purchaseMsg.setContent(bookISBN);

        System.out.println("Customer is requesting to purchase book with ISBN: " + bookISBN);

        // Send the message
        customerAgent.send(purchaseMsg);

        // Wait for a reply
        MessageTemplate mt = MessageTemplate.or(
                MessageTemplate.MatchPerformative(ACLMessage.INFORM),
                MessageTemplate.MatchPerformative(ACLMessage.FAILURE));

        ACLMessage reply = customerAgent.blockingReceive(mt, 5000); // Wait up to 5 seconds

        if (reply != null) {
            if (reply.getPerformative() == ACLMessage.INFORM) {
                // Purchase successful
                System.out.println("Customer purchased book successfully!");
            } else {
                // Purchase failed
                System.out.println("Customer couldn't purchase book: " + reply.getContent());
            }
        } else {
            // No reply received (timeout)
            System.out.println("Customer received no purchase reply from bookstore!");
        }
    }
}