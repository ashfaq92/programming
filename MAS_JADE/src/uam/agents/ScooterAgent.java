package uam.agents;

import jade.core.Agent;

import uam.behaviors.scooter.HandleDispatchRequest;

public class ScooterAgent extends Agent {
    public boolean busy = false;

    protected void setup() {
        System.out.println(getLocalName() + ": Ready for dispatch requests.");
        addBehaviour(new HandleDispatchRequest(this));
    }
}