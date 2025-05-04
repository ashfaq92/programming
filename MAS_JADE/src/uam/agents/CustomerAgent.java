package uam.agents;

import jade.core.Agent;

import uam.behaviors.customer.RequestJourney;
import uam.behaviors.customer.ReceiveProposal;

public class CustomerAgent extends Agent {
    protected void setup() {
        System.out.println(getLocalName() + ": Ready to request a journey.");
        addBehaviour(new RequestJourney(this));
        addBehaviour(new ReceiveProposal(this));
    }
}