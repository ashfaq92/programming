# Define Functionality: F = {f, t, p}
class Functionality:
    def __init__(self, func, t: float, performance:float):
        """
        func: a callable mapping conditions -> effects
        t: execution time
        performance: probability of success [0, 1]
        """
        self.f = func
        self.t = t
        self.p = performance

    def __repr__(self):
        return f"Functionality(f={self.f}, t={self.t}, p={self.p})"