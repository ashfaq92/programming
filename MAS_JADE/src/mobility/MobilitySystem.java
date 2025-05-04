package mobility;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

/**
 * Main class to launch the mobility multi-agent system
 */
public class MobilitySystem {
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
            // Create the supervisor agent
            AgentController supervisorAgent = container.createNewAgent(
                    "supervisor",
                    "mobility.agents.SupervisorSoSAgent",
                    null);

            // Create the scooter supervisor agent
            AgentController scooterSupervisorAgent = container.createNewAgent(
                    "scooterSupervisor",
                    "mobility.agents.ScooterSupervisorAgent",
                    null);

            // Create scooter agents
            AgentController scooter1 = container.createNewAgent(
                    "scooter1",
                    "mobility.agents.ScooterAgent",
                    null);

            AgentController scooter2 = container.createNewAgent(
                    "scooter2",
                    "mobility.agents.ScooterAgent",
                    null);

            // Create customer agents with different start and destination locations
            AgentController customer1 = container.createNewAgent(
                    "customer1",
                    "mobility.agents.CustomerAgent",
                    new Object[] { 1.0, 1.0, 8.0, 8.0 } // start_x, start_y, dest_x, dest_y
            );

            AgentController customer2 = container.createNewAgent(
                    "customer2",
                    "mobility.agents.CustomerAgent",
                    new Object[] { 2.0, 3.0, 9.0, 7.0 });

            // Start the agents
            System.out.println("Starting agents...");

            // Start coordinator agents first
            supervisorAgent.start();
            Thread.sleep(1000);

            scooterSupervisorAgent.start();
            Thread.sleep(1000);

            // Start resource agents
            scooter1.start();
            scooter2.start();
            Thread.sleep(1000);

            // Start customer agents
            customer1.start();
            Thread.sleep(2000); // Delay to see interactions more clearly
            customer2.start();

            System.out.println("All agents started.");

        } catch (StaleProxyException | InterruptedException e) {
            e.printStackTrace();
        }
    }
}