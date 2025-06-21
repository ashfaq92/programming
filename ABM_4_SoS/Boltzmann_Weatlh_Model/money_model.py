import mesa
import utils

class MoneyAgent(mesa.Agent):
    """An agent with fixed initial wealth"""
    def __init__(self, model, ethnicity):
        # pass the parameters to the parent class
        super().__init__(model)
        # create agent's variable and set the initial values
        self.wealth = 1
        self.ethnicity = ethnicity

    def move(self):
        possible_steps = self.model.grid.get_neighborhood(
            self.pos,
            moore=True,
            include_center=False)
        new_position = self.random.choice(possible_steps)
        self.model.grid.move_agent(self, new_position)
    
    def give_money(self, similars):
        if self.wealth > 0:
            other_agent = self.random.choice(similars)
            other_agent.wealth += 1
            self.wealth -= 1



class MoneyModel(mesa.Model):
    """A model with fixed number of agents"""
    def __init__(self, n, width, height, seed=None):
        super().__init__(seed=seed)
        self.num_agents = n

        # Create a list of our different ethnicities
        ethnicities = ["Green", "Blue", "Mixed"]

        # Create agents
        MoneyAgent.create_agents(model=self, n=self.num_agents, ethnicity=self.random.choice(ethnicities))

        # collect the output
        self.datacollector = mesa.DataCollector(
            model_reporters={"Gini": utils.compute_gini},
            agent_reporters={"Wealth": "wealth", "Ethnicity": "ethnicity"}
        )
    
    def step(self):
        """Advance the model by one step"""
        # Collect data
        self.datacollector.collect(self)

        # group agents based on ethnicity
        grouped_agents = self.agents.groupby("ethnicity")
        print(grouped_agents)
        for ethnic, similars in grouped_agents:
            if ethnic != "Mixed":
                similars.shuffle_do("give_money", similars)
            else:
                similars.shuffle_do("give_money", self.agents)




