from robot import Robot


class CooperativeRobot(Robot):
    """Robot that cooperates with others"""

    def step(self):
        """Cooperative decision-making"""
        # For now, use basic behavior
        if self.energy <= 0:
            return

        # Basic behavior without swapping
        self._basic_step()

    def choose_target_box(self):
        """Choose box considering team benefit"""
        return self.find_nearest_box()  # Placeholder

    def _basic_step(self):
        """Basic step without advanced behaviors"""
        # ... basic robot behavior ...