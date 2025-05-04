package bookstore.behaviors.bookstore;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import bookstore.agents.BookstoreAgent;
import bookstore.models.Book;

/**
 * Behavior for handling book query requests
 */
public class HandleQueryBehavior extends CyclicBehaviour {
    private BookstoreAgent bookstoreAgent;

    /**
     * Constructor
     * 
     * @param agent The bookstore agent
     */
    public HandleQueryBehavior(BookstoreAgent agent) {
        this.bookstoreAgent = agent;
    }

    /**
     * Method called at each behavior execution
     */
    @Override
    public void action() {
        // Create a template to filter only QUERY messages
        MessageTemplate mt = MessageTemplate.MatchPerformative(ACLMessage.QUERY_REF);
        ACLMessage message = bookstoreAgent.receive(mt);

        if (message != null) {
            // Message received
            String title = message.getContent();
            System.out.println("Bookstore received query for book with title: " + title);

            // Search for books matching the title
            Book[] books = bookstoreAgent.searchBooksByTitle(title);

            // Prepare reply
            ACLMessage reply = message.createReply();

            if (books.length > 0) {
                // Books found
                reply.setPerformative(ACLMessage.INFORM);

                // For simplicity, we'll just send the ISBN of the first matching book
                // In a real system, we might send all matches or more detailed information
                reply.setContent(books[0].getIsbn());
                System.out.println(
                        "Bookstore found book: " + books[0].getTitle() + " (ISBN: " + books[0].getIsbn() + ")");
            } else {
                // No books found
                reply.setPerformative(ACLMessage.FAILURE);
                reply.setContent("no-such-book");
                System.out.println("Bookstore couldn't find any book matching title: " + title);
            }

            // Send the reply
            bookstoreAgent.send(reply);
        } else {
            // If no message is received, block until one arrives
            block();
        }
    }
}