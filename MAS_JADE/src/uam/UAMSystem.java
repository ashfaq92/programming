package uam;


import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

/**
 * Main launcher for the Urban Air Mobility multi-agent system
 */
public class UAMSystem {
    public static void main(String[] args) {
        // Get a hold on JADE runtime
        Runtime rt = Runtime.instance();

        // Create a default profile
        Profile profile = new ProfileImpl();
        profile.setParameter(Profile.GUI, "true"); // Launch the JADE GUI
        profile.setParameter(Profile.MAIN_HOST, "localhost");

        // Create the main container
        AgentContainer container = rt.createMainContainer(profile);

        try {
            // Create and launch SupervisorSoSAgent
            AgentController supervisor = container.createNewAgent(
                    "supervisor",
                    "uam.agents.SupervisorSoSAgent",
                    null);

            // Create and launch ScooterAgent
            AgentController scooter = container.createNewAgent(
                    "scooter1",
                    "uam.agents.ScooterAgent",
                    null);

            // Create and launch CustomerAgent
            AgentController customer = container.createNewAgent(
                    "customer",
                    "uam.agents.CustomerAgent",
                    null);

            System.out.println("[System] Starting UAM agents...");

            supervisor.start();
            Thread.sleep(500); // allow supervisor to boot first

            scooter.start();
            Thread.sleep(500); // give scooter time too

            customer.start();

            System.out.println("[System] All agents started.");

        } catch (StaleProxyException | InterruptedException e) {
            e.printStackTrace();
        }
    }
}
