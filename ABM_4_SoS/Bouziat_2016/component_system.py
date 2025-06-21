from goals import GoalResource
from functionality import Functionality
import json
from resource import Resource

class ComponentSystem:
    """
    Notes: Need to add resources BEFORE adding resource goals
    """
    def __init__(self, name):
        self.name = name
        self.F = set()  # set of functionalities
        self.R = set()  # set of resources
        self.Grs = set()  # set of resource goals
        self.L = set()  # set of links (neighboring systems)
        self.criticality_history = []
        self.current_time = 0

    def add_functionality(self, functionality):
        if isinstance(functionality, Functionality):
            self.F.add(functionality)
        else:
            raise ValueError(f'{functionality} must be a Functionality object')

    def add_resource(self, resource):
        if isinstance(resource, Resource):
            self.R.add(resource)
        else:
            raise ValueError(f'{resource} must be a Resource object')

    def get_resource_amount(self, resource) -> float:
        for res in self.R:
            if res.type == resource.type:
                return res.quantity
        print("Error in getting resource amount!")
        return 0

    def add_goal_resource(self, goal):
        # check if the resource exists or not
        for res in self.R:
            if res.type == goal.type:
                self.Grs.add(goal)
                break
        else:
            raise ValueError(f"ERROR: No corresponding resource exists against `{goal.type}`!")

    def add_link(self, link_system):
        # check if itself
        if link_system.name == self.name:
            raise ValueError('Cannot add link to itself!')
        else:
            self.L.add(link_system)

    def initialize_criticality(self, a=1.0):
        """Call this after adding resources and goals"""
        if not self.criticality_history:
            initial_criticality = self.calculate_criticality(a)
            self.criticality_history.append(initial_criticality)

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

                    # print(f"{gr.type}: gap={res_gap:.1f}, criticality={goal_criticality:.4f}, priority={gr.priority}")  

        # calculate weighted average
        overall_criticality = weighted_sum / total_priority if total_priority > 0 else 0.0
        return overall_criticality

    def get_criticality(self, time=None):
        """Get criticality from history"""
        if not self.criticality_history:
            return 0.0
        if time is None:
            return self.criticality_history[-1]  # the last updated criticality
        return self.criticality_history[min(time, len(self.criticality_history) - 1)]

    def update_criticality(self, new_criticality):
        """Add new criticality to history and advance time"""
        self.criticality_history.append(new_criticality)
        self.current_time += 1

    def calculate_anticipated_criticality(self, system_sj, functionality_f, target_time):
        """
        Algorithm 2: Calculate anticipated criticality using linear extrapolation
        """
        # Get current and previous criticality
        current_crit = system_sj.get_criticality()
        previous_crit = system_sj.get_criticality(max(0, system_sj.current_time - 1))
        
        # Calculate slope (a) and intercept (b)
        b = previous_crit
        t0 = max(0, system_sj.current_time - 1)
        
        if not functionality_f.has_effect_on_system(system_sj):
            # No effect, use natural trend
            a = current_crit - previous_crit
        else:
            # Include effect of functionality
            effect = functionality_f.calculate_effect(system_sj)
            a = (current_crit + effect) - previous_crit
        
        # Linear extrapolation: C'(t) = a * (t - t0) + b
        time_diff = target_time - t0
        anticipated_crit = a * time_diff + b
        return max(0.0, min(1.0, anticipated_crit))  # clamp to [0,1]

    def cooperative_decision(self):
        """
        Algorithm 1: Cooperative component-system decision
        """
        if not self.F:
            return None
        
        functionality_deltas = {}  # f -> {neighbor: delta}
        
        # For each functionality
        for f in self.F:
            deltas_f = {}
            t_f = self.current_time + f.duration  # calculate final time
            
            # For each neighbor
            for sj in self.L:
                # Calculate anticipated criticality
                anticipated_crit = self.calculate_anticipated_criticality(sj, f, t_f)
                current_crit = sj.get_criticality()
                
                # Calculate delta (difference in criticality)
                delta = anticipated_crit - current_crit
                deltas_f[sj] = delta
            
            functionality_deltas[f] = deltas_f
        
        # Find the functionality that minimizes the maximum criticality increase
        best_f = None
        min_max_delta = float('inf')
        
        for f, deltas in functionality_deltas.items():
            if deltas:  # if there are neighbors
                max_delta = max(deltas.values())
                if max_delta < min_max_delta:
                    min_max_delta = max_delta
                    best_f = f
        
        # Find comparable actions (F_10%)
        if best_f and functionality_deltas:
            all_deltas = [delta for deltas in functionality_deltas.values() for delta in deltas.values()]
            min_delta = min(all_deltas) if all_deltas else 0
            threshold = 0.1 * abs(min_delta)  # 10% threshold
            
            comparable_funcs = []
            for f, deltas in functionality_deltas.items():
                if deltas:
                    max_delta = max(deltas.values())
                    if abs(max_delta - min_max_delta) <= threshold:
                        comparable_funcs.append(f)
            
            # Among comparable actions, choose the fastest one
            if comparable_funcs:
                fastest_f = min(comparable_funcs, key=lambda f: f.duration)
                return fastest_f
        
        return best_f

    def cooperative_decision_debug(self):
        """
        Debug version of cooperative decision with detailed output
        """
        if not self.F:
            return None
        
        print(f"\n--- {self.name} Decision Process ---")
        functionality_deltas = {}
        
        # For each functionality
        for f in self.F:
            print(f"Evaluating {f.name}:")
            deltas_f = {}
            t_f = self.current_time + f.duration
            
            # For each neighbor
            for sj in self.L:
                anticipated_crit = self.calculate_anticipated_criticality(sj, f, t_f)
                current_crit = sj.get_criticality()
                delta = anticipated_crit - current_crit
                deltas_f[sj] = delta
                print(f"  {sj.name}: current={current_crit:.3f}, anticipated={anticipated_crit:.3f}, delta={delta:.3f}")
            
            if deltas_f:
                max_delta = max(deltas_f.values())
                print(f"  Max delta for {f.name}: {max_delta:.3f}")
            functionality_deltas[f] = deltas_f
        
        # Find best functionality
        best_f = None
        min_max_delta = float('inf')
        
        for f, deltas in functionality_deltas.items():
            if deltas:
                max_delta = max(deltas.values())
                if max_delta < min_max_delta:
                    min_max_delta = max_delta
                    best_f = f
        
        print(f"Best functionality: {best_f.name if best_f else 'None'} (min_max_delta: {min_max_delta:.3f})")
        return best_f

    def __repr__(self, verbose=False):
        output = {}
        if verbose:
            output["CS Name"] = self.name
            output["Functionalities"] = [repr(f) for f in self.F]
            output["Resources"] = [repr(r) for r in self.R]
            output["ResourceGoals"] = [repr(gr) for gr in self.Grs]
            output["Links"] = [link.name for link in self.L]
        else:
            output["CS Name"] = self.name
            output["Functionalities"] = len(self.F)
            output["Resources"] = len(self.R)
            output["ResourceGoals"] = len(self.Grs)
            output["Links"] = len(self.L)
        return json.dumps(output, indent=4)