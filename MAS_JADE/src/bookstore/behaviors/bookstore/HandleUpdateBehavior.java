package bookstore.behaviors.bookstore;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.BookstoreAgent;
import bookstore.models.Book;

/**
 * Behavior for handling inventory update requests
 */
public class HandleUpdateBehavior extends CyclicBehaviour {
    private BookstoreAgent bookstoreAgent;

    /**
     * Constructor
     * 
     * @param agent The bookstore agent
     */
    public HandleUpdateBehavior(BookstoreAgent agent) {
        this.bookstoreAgent = agent;
    }

    /**
     * Method called at each behavior execution
     */
    @Override
    public void action() {
        // Create a template to filter only PROPOSE messages (for updates)
        MessageTemplate mt = MessageTemplate.MatchPerformative(ACLMessage.PROPOSE);
        ACLMessage message = bookstoreAgent.receive(mt);

        if (message != null) {
            // Message received
            String content = message.getContent();
            System.out.println("Bookstore received inventory update: " + content);

            // The message format is expected to be "ISBN:quantity"
            String[] parts = content.split(":");
            if (parts.length == 2) {
                String isbn = parts[0];
                try {
                    int quantity = Integer.parseInt(parts[1]);

                    // Update the book quantity
                    boolean updated = bookstoreAgent.updateBookQuantity(isbn, quantity);

                    // Prepare reply
                    ACLMessage reply = message.createReply();

                    if (updated) {
                        // Update successful
                        reply.setPerformative(ACLMessage.AGREE);
                        reply.setContent("update-successful");

                        Book book = bookstoreAgent.getBook(isbn);
                        System.out.println("Bookstore updated quantity for book: " + book.getTitle()
                                + " (new quantity: " + quantity + ")");
                    } else {
                        // Book not found
                        reply.setPerformative(ACLMessage.REFUSE);
                        reply.setContent("book-not-found");
                        System.out.println("Bookstore couldn't update book with ISBN: " + isbn + " (not found)");
                    }

                    // Send the reply
                    bookstoreAgent.send(reply);
                } catch (NumberFormatException e) {
                    System.err.println("Invalid quantity format in update message.");
                }
            } else {
                System.err.println("Invalid update message format. Expected 'ISBN:quantity'.");
            }
        } else {
            // If no message is received, block until one arrives
            block();
        }
    }
}