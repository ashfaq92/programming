import math

class GoalResource:
    def __init__(self, type: str, value: float, kind: str, priority: int):
        self.type = type    # goal name basically
        self.value = value
        self.kind = kind    # Goal type "=" for EQ, "!=" or "≠" for NEQ
        self.priority = priority

    def calculate_goal_criticality(self, gap, a):
        """
        Calculate criticality for a single goal based on resource gap

        Args:
            gap: Resource gap (+ve = need more, -ve = have excess)
            a: steepness parameter (smaller=sharper transition)
        Returns:
             criticality value between 0 and 2 (typically 0-1)
        """
        try:
            if self.kind == "EQ":
                # Equality goal: want exactly the right amount
                term1 = 1 / math.exp(gap + a)
                term2 = 1 / math.exp(gap - a)
                goal_criticality = 1 + term1 - term2
            elif self.kind == "NEQ":
                # Inequality goal: want at least this much
                term1 = 1 / math.exp(gap + a)
                term2 = 1 / math.exp(gap - a)
                goal_criticality = -term1 + term2  # TODO: double check
            else:
                raise ValueError(f"Unknown goal kind: {self.kind}")
            # return max(0, goal_criticality)
            return max(0.0, min(1.0, goal_criticality))
        except OverflowError:
            # handle extreme values
            return 0.0 if gap < -10 else 1.0

    def __repr__(self):
        return f"GoalResource(type={self.type}, value={self.value}, kind={self.kind}, priority={self.priority})"


# class GoalLink:
#     def __init__(self, target_system_id: str):
#         self.target_system_id = target_system_id
#
#     def __repr__(self):
#         return f"GoalLink(to={self.target_system_id})"

class GoalLink:
    def __init__(self, target_system: str): # TODO: target_system or target_system_id
        self.Sj = target_system

    def __repr__(self):
        return f"GL(to={self.Sj})"