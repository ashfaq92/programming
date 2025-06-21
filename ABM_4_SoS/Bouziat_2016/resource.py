class Resource:
    def __init__(self, type: str, quantity: float):
        self.type = type
        self.quantity = quantity

    def __repr__(self):
        return f"Resource(type={self.type}, quantity={self.quantity})"
