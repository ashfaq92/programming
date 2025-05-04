package bookstore.behaviors.customer;

import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.CustomerAgent;

/**
 * Behavior for querying for a book by title
 */
public class QueryBookBehavior extends OneShotBehaviour {
    private CustomerAgent customerAgent;
    private String bookTitle;

    /**
     * Constructor
     * 
     * @param agent The customer agent
     * @param title The title of the book to query
     */
    public QueryBookBehavior(CustomerAgent agent, String title) {
        this.customerAgent = agent;
        this.bookTitle = title;
    }

    /**
     * Method called when the behavior is executed
     */
    @Override
    public void action() {
        // Check if we have a bookstore agent to talk to
        if (customerAgent.getBookstoreAgent() == null) {
            System.out.println("Customer can't query for books: no bookstore found!");
            return;
        }

        // Create a query message
        ACLMessage queryMsg = new ACLMessage(ACLMessage.QUERY_REF);
        queryMsg.addReceiver(customerAgent.getBookstoreAgent());
        queryMsg.setContent(bookTitle);

        System.out.println("Customer is querying for book: " + bookTitle);

        // Send the message
        customerAgent.send(queryMsg);

        // Wait for a reply
        MessageTemplate mt = MessageTemplate.or(
                MessageTemplate.MatchPerformative(ACLMessage.INFORM),
                MessageTemplate.MatchPerformative(ACLMessage.FAILURE));

        ACLMessage reply = customerAgent.blockingReceive(mt, 5000); // Wait up to 5 seconds

        if (reply != null) {
            if (reply.getPerformative() == ACLMessage.INFORM) {
                // Book found, we received the ISBN
                String isbn = reply.getContent();
                System.out.println("Customer found book with ISBN: " + isbn);

                // Store the ISBN for purchase
                customerAgent.setTargetBookISBN(isbn);
            } else {
                // Book not found
                System.out.println("Customer couldn't find book: " + bookTitle);
            }
        } else {
            // No reply received (timeout)
            System.out.println("Customer received no reply from bookstore!");
        }
    }
}