package bookstore.behaviors.bookstore;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.BookstoreAgent;
import bookstore.models.Book;

/**
 * Behavior for handling book purchase requests
 */
public class HandlePurchaseBehavior extends CyclicBehaviour {
    private BookstoreAgent bookstoreAgent;

    /**
     * Constructor
     * 
     * @param agent The bookstore agent
     */
    public HandlePurchaseBehavior(BookstoreAgent agent) {
        this.bookstoreAgent = agent;
    }

    /**
     * Method called at each behavior execution
     */
    @Override
    public void action() {
        // Create a template to filter only REQUEST messages (for purchase)
        MessageTemplate mt = MessageTemplate.MatchPerformative(ACLMessage.REQUEST);
        ACLMessage message = bookstoreAgent.receive(mt);

        if (message != null) {
            // Message received
            String isbn = message.getContent();
            System.out.println("Bookstore received purchase request for ISBN: " + isbn);

            // Try to sell the book
            boolean sold = bookstoreAgent.sellBook(isbn);

            // Prepare reply
            ACLMessage reply = message.createReply();

            if (sold) {
                // Book sold successfully
                reply.setPerformative(ACLMessage.INFORM);
                reply.setContent("purchase-successful");

                Book book = bookstoreAgent.getBook(isbn);
                System.out.println(
                        "Bookstore sold book: " + book.getTitle() + " (remaining: " + book.getQuantity() + ")");
            } else {
                // Book not available
                reply.setPerformative(ACLMessage.FAILURE);
                reply.setContent("out-of-stock");
                System.out.println("Bookstore couldn't sell book with ISBN: " + isbn + " (out of stock)");
            }

            // Send the reply
            bookstoreAgent.send(reply);
        } else {
            // If no message is received, block until one arrives
            block();
        }
    }
}