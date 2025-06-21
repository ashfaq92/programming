class Rule:
    def __init__(self, conditions, effects):
        self.conditions = conditions
        self.effects = effects

    def __str__(self):
        return f"Rule({self.conditions} → {self.effects})"