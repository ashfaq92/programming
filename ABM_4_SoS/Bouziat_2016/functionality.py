# Define Functionality: F = {f, t, p}
class Functionality:
    # def __init__(self, name: str, func, t: float, performance:float):
    def __init__(self, name: str, duration: float, resource_effects: dict = None):
        """
        func: a callable mapping conditions -> effects
        duration: execution time (t) [how long this functionality will take]
        performance: probability of success [0, 1]
        """
        self.name = name
        # self.f = func
        self.duration = duration
        # self.p = performance
        self.resource_effects = resource_effects or {}  # {resource_type: change_amount}

    def __repr__(self):
        return f"Functionality(name={self.name}, duration={self.duration})"

    def has_effect_on_system(self, system):
        """
        Check if this functionality affects the given system
        """
        # todo: for now, simple check - if system has resources this functionality affects
        for resource_type in self.resource_effects.keys():
            for resource in system.R:
                if resource.type == resource_type:
                    return True
        return False
    
    def calculate_effect(self, system):
        """
        calculate the effect of this functionality on system's criticality
        """
        if not self.has_effect_on_system(system):
            return 0.0
        
        # todo: simple effect - sum of resource changes weighted by current criticality
        total_effect = 0.0

        for resource_type, change in self.resource_effects.items():
            for resource in system.R:
                if resource.type == resource_type:
                    # Positive change reduces criticality, negative increases it
                    total_effect -= change * 0.01   #todo: simple sclaing factor
            
        return total_effect