package bookstore.agents;

import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;

import java.util.HashMap;
import java.util.Map;

import bookstore.behaviors.bookstore.HandleQueryBehavior;
import bookstore.behaviors.bookstore.HandlePurchaseBehavior;
import bookstore.behaviors.bookstore.HandleUpdateBehavior;
import bookstore.models.Book;

/**
 * Agent representing a bookstore that manages inventory and handles customer
 * requests
 */
public class BookstoreAgent extends Agent {
    // Store inventory as a map of ISBN to Book
    private Map<String, Book> inventory = new HashMap<>();

    /**
     * Setup method called when the agent is initialized
     */
    @Override
    protected void setup() {
        System.out.println("Bookstore agent " + getLocalName() + " is ready.");

        // Initialize the bookstore with some books
        initializeInventory();

        // Register the agent with the DF (Directory Facilitator)
        registerWithDF();

        // Add behaviors to handle different types of messages
        addBehaviour(new HandleQueryBehavior(this));
        addBehaviour(new HandlePurchaseBehavior(this));
        addBehaviour(new HandleUpdateBehavior(this));
    }

    /**
     * Method to initialize the bookstore inventory with some sample books
     */
    private void initializeInventory() {
        // Add sample books to inventory
        addBook(new Book("The Great Gatsby", "F. Scott Fitzgerald", "978-0743273565", 9.99, 5));
        addBook(new Book("To Kill a Mockingbird", "Harper Lee", "978-0061120084", 8.99, 3));
        addBook(new Book("1984", "George Orwell", "978-0451524935", 7.99, 7));
        addBook(new Book("Pride and Prejudice", "Jane Austen", "978-0141439518", 6.99, 2));
        addBook(new Book("The Catcher in the Rye", "J.D. Salinger", "978-0316769488", 8.49, 4));

        System.out.println("Initialized inventory with " + inventory.size() + " books.");
    }

    /**
     * Method to register the agent with the Directory Facilitator
     */
    private void registerWithDF() {
        // Create a description for this agent
        DFAgentDescription dfd = new DFAgentDescription();
        dfd.setName(getAID());

        // Create a service description for book querying
        ServiceDescription queryService = new ServiceDescription();
        queryService.setType("book-query");
        queryService.setName("book-querying");

        // Create a service description for book selling
        ServiceDescription sellService = new ServiceDescription();
        sellService.setType("book-selling");
        sellService.setName("book-purchasing");

        // Create a service description for inventory management
        ServiceDescription inventoryService = new ServiceDescription();
        inventoryService.setType("inventory-management");
        inventoryService.setName("inventory-updating");

        // Add all services to the agent description
        dfd.addServices(queryService);
        dfd.addServices(sellService);
        dfd.addServices(inventoryService);

        try {
            // Register with the DF
            DFService.register(this, dfd);
            System.out.println("Bookstore agent registered with DF.");
        } catch (FIPAException e) {
            e.printStackTrace();
            System.err.println("Failed to register with DF: " + e.getMessage());
        }
    }

    /**
     * Called when the agent is terminated
     */
    @Override
    protected void takeDown() {
        // Deregister from the DF
        try {
            DFService.deregister(this);
            System.out.println("Bookstore agent deregistered from DF.");
        } catch (FIPAException e) {
            e.printStackTrace();
        }

        System.out.println("Bookstore agent " + getLocalName() + " terminating.");
    }

    /**
     * Add a book to the inventory
     * 
     * @param book The book to add
     */
    public void addBook(Book book) {
        inventory.put(book.getIsbn(), book);
    }

    /**
     * Get a book by its ISBN
     * 
     * @param isbn The ISBN of the book
     * @return The book, or null if not found
     */
    public Book getBook(String isbn) {
        return inventory.get(isbn);
    }

    /**
     * Search for books by title (partial match)
     * 
     * @param title The title to search for
     * @return An array of matching books
     */
    public Book[] searchBooksByTitle(String title) {
        return inventory.values().stream()
                .filter(book -> book.getTitle().toLowerCase().contains(title.toLowerCase()))
                .toArray(Book[]::new);
    }

    /**
     * Get the entire inventory
     * 
     * @return The inventory map
     */
    public Map<String, Book> getInventory() {
        return inventory;
    }

    /**
     * Sell a book by decreasing its quantity
     * 
     * @param isbn The ISBN of the book to sell
     * @return true if the book was available and sold, false otherwise
     */
    public boolean sellBook(String isbn) {
        Book book = inventory.get(isbn);
        if (book != null) {
            return book.sellOne();
        }
        return false;
    }

    /**
     * Update the quantity of a book
     * 
     * @param isbn     The ISBN of the book
     * @param quantity The new quantity
     * @return true if updated successfully, false if book not found
     */
    public boolean updateBookQuantity(String isbn, int quantity) {
        Book book = inventory.get(isbn);
        if (book != null) {
            book.setQuantity(quantity);
            return true;
        }
        return false;
    }
}