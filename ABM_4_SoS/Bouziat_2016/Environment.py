class Environment:
    def __init__(self):
        self.E = set()  # Set of entities
        self.Rules = set()  # Set of rules

    def add_entity(self, entity):
        self.E.add(entity)

    def add_rule(self, rule):
        self.Rules.add(rule)

    def __str__(self):
        entities = [e.entity_id for e in self.E]
        return f"Environment = {{{entities}, {len(self.Rules)} rules}}"