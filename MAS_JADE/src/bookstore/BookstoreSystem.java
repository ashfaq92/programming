package bookstore;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

/**
 * Main class to launch the bookstore multi-agent system
 */
public class BookstoreSystem {
    public static void main(String[] args) {
        // Get a hold on JADE runtime
        Runtime rt = Runtime.instance();

        // Create a default profile
        Profile profile = new ProfileImpl();
        profile.setParameter(Profile.MAIN_HOST, "localhost");
        profile.setParameter(Profile.GUI, "true"); // Start the JADE GUI

        // Create a main container
        AgentContainer container = rt.createMainContainer(profile);

        try {
            // Create the bookstore agent
            AgentController bookstoreAgent = container.createNewAgent(
                    "bookstore",
                    "bookstore.agents.BookstoreAgent",
                    null);

            // Create the manager agent
            AgentController managerAgent = container.createNewAgent(
                    "manager",
                    "bookstore.agents.ManagerAgent",
                    null);

            // Create a customer agent looking for "The Great Gatsby"
            AgentController customer1 = container.createNewAgent(
                    "customer1",
                    "bookstore.agents.CustomerAgent",
                    new Object[] { "The Great Gatsby" });

            // Create another customer agent looking for "1984"
            AgentController customer2 = container.createNewAgent(
                    "customer2",
                    "bookstore.agents.CustomerAgent",
                    new Object[] { "1984" });

            // Create a third customer agent looking for a book that doesn't exist
            AgentController customer3 = container.createNewAgent(
                    "customer3",
                    "bookstore.agents.CustomerAgent",
                    new Object[] { "Harry Potter" });

            // Start the agents
            System.out.println("Starting agents...");
            bookstoreAgent.start();

            // Give the bookstore agent time to initialize and register with the DF
            Thread.sleep(1000);

            managerAgent.start();
            customer1.start();
            customer2.start();
            customer3.start();

            System.out.println("All agents started.");

        } catch (StaleProxyException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}