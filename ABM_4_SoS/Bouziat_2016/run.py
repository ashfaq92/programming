from ComponentSystem import ComponentSystem
from Resource import Resource
from Goals import GoalResource
import helper_funcs
import json

# Example usage with criticality
if __name__ == "__main__":
    SENSITIVITY = 5.0
    print("=== CRITICALITY METRIC DEMO ===\n")
    
    # Create a robot system
    robot_system = ComponentSystem("Robot2")
    
    # Add resources
    energy_resource = Resource("energy", 10.0)
    robot_system.add_resource(energy_resource)

    health_resource = Resource("health", 85.0)
    robot_system.add_resource(health_resource)

    # print(robot_system, verbose=True)
    print(robot_system.__repr__(verbose=True))

    # Add goals
    energy_goal = GoalResource("energy", 80, "EQ", 2)  # get energy level at least 80%
    robot_system.add_resource_goal(energy_goal)

    health_goal = GoalResource("health", 90, "EQ", 2)
    robot_system.add_resource_goal(health_goal)

    print(robot_system.__repr__(verbose=True))
    # calculating criticality
    criticality = robot_system.calculate_criticality(SENSITIVITY)
    print(f"overall criticality: {criticality}")
