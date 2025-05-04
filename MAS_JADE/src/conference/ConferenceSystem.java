package conference;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

/**
 * Main class to launch the conference multi-agent system
 */
public class ConferenceSystem {
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
            // Create the chair agent
            AgentController chairAgent = container.createNewAgent(
                    "chair",
                    "conference.agents.ChairAgent",
                    null);

            // Create author agents
            AgentController authorAgent1 = container.createNewAgent(
                    "author1",
                    "conference.agents.AuthorAgent",
                    null);

            AgentController authorAgent2 = container.createNewAgent(
                    "author2",
                    "conference.agents.AuthorAgent",
                    null);

            // Start the agents
            System.out.println("Starting conference agents...");

            // Start chair agent first
            chairAgent.start();

            // Give the chair agent time to initialize
            Thread.sleep(1000);

            // Start author agents
            authorAgent1.start();
            authorAgent2.start();

            System.out.println("All conference agents started.");

        } catch (StaleProxyException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}