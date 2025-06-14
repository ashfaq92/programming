from Goals import GoalResource
import json


class ComponentSystem:
    """
    Notes: Need to add resources BEFORE adding resource goals
    """
    def __init__(self, name):
        self.name = name
        self.F = set()  # set of functionalities
        self.R = set()  # set of resources
        self.Grs = set()  # set of resource goals
        self.L = set()  # set of links

    def add_functionality(self, functionality):
        self.F.add(functionality)

    def add_resource(self, resource):
        self.R.add(resource)

    def get_resource_amount(self, resource) -> float:
        for res in self.R:
            if res.type == resource.type:
                return res.quantity
        print("Error in getting resource amount!")
        return 0

    def add_resource_goal(self, goal):
        # check if the resource exists or not
        for res in self.R:
            if res.type == goal.type:
                self.Grs.add(goal)
                break
        else:
            raise ValueError(f"ERROR: No corresponding resource exists against `{goal.type}`!")

    def add_links(self, link):
        self.L.add(link)


    def calculate_criticality(self, a=1.0):
        """
            Calculate overall criticality for an agent based on all its goals
            Args:
                @param component_system: Component system with goals
                @param resource_type: type of resource to consider
                @param a: steepness parameter for sigmoid
            Returns:
                Overall criticality (0-1 range)
            """
        if not self.Grs:
            return 0.0

        weighted_sum = 0.0
        total_priority = 0.0
        print(f"Calculating criticality for {self.name}:")
        for res in self.R:
            for gr in self.Grs:
                if res.type == gr.type:
                    # calculate resource gap for this goal
                    res_gap = gr.value - res.quantity

                    # calculate the criticality for this goal
                    goal_criticality = gr.calculate_goal_criticality(res_gap, a)

                    # add to weighted sum
                    weighted_sum += goal_criticality * gr.priority
                    total_priority += gr.priority

                    print(f"{gr.type}: gap={res_gap:.1f}, criticality={goal_criticality:.4f}, priority={gr.priority}")

        # calculate weighted average
        overall_criticality = weighted_sum / total_priority if total_priority > 0 else 0.0
        return overall_criticality

    def __repr__(self, verbose=False):
        output = {}
        if verbose:
            output["CS Name"] = self.name
            output["Functionalities"] = [repr(f) for f in self.F]
            output["Resources"] = [repr(r) for r in self.R]
            output["ResourceGoals"] = [repr(gr) for gr in self.Grs]
        else:
            output["CS Name"] = self.name
            output["Functionalities"] = (len(self.F))
            output["Resources"] = len(self.R)
            output["ResourceGoals"] = len(self.Grs)
            return output
        return json.dumps(output, indent=4)