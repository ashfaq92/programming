from robot import Robot


class CooperativeRobot(Robot):
    """Robot that cooperates with others"""

    def step(self):
        """Cooperative decision-making"""
        # TODO: Implement when you get the specs
        # For now, use basic behavior
        if self.energy <= 0:
            return

        # Basic behavior without swapping
        self._basic_step()

    def choose_target_box(self):
        """Choose box considering team benefit"""
        # TODO: Implement cooperative selection
        return self.find_nearest_box()  # Placeholder

    def _basic_step(self):
        """Basic step without advanced behaviors"""
        self.update_states()
        # ... basic robot behavior ...