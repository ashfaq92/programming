from component_system import ComponentSystem
from resource import Resource
from goals import GoalResource
from functionality import Functionality

def create_demo_scenario():
    """Create a demo scenario with multiple cooperating systems"""
    
    # Create three robot systems
    robot1 = ComponentSystem("Robot_Alpha")
    robot2 = ComponentSystem("Robot_Beta") 
    robot3 = ComponentSystem("Robot_Gamma")
    
    # Add resources to each robot
    for robot, energy, health in [(robot1, 30, 85), (robot2, 70, 60), (robot3, 90, 40)]:
        robot.add_resource(Resource("energy", energy))
        robot.add_resource(Resource("health", health))
        
        # Add goals
        robot.add_goal_resource(GoalResource("energy", 80, "EQ", 2))
        robot.add_goal_resource(GoalResource("health", 90, "EQ", 2))
    
    # Add functionalities
    # Energy sharing functionality
    energy_share = Functionality("share_energy", 2.0, {"energy": 10})  # gives 10  energy, takes 2 time units
    repair_func = Functionality("repair", 5.0, {"health": 15})  # gives 15 health, takes 5 time units
    scout_func = Functionality("scout", 1.0, {"energy": -5})  # costs 5 energy, fast action
    
    for robot in [robot1, robot2, robot3]:
        robot.add_functionality(energy_share)
        robot.add_functionality(repair_func)
        robot.add_functionality(scout_func)
    
    # Create neighborhood links
    robot1.add_link(robot2)
    robot1.add_link(robot3)
    robot2.add_link(robot1)
    robot2.add_link(robot3)
    robot3.add_link(robot1)
    robot3.add_link(robot2)
    
    return [robot1, robot2, robot3]

if __name__ == "__main__":
    print("=== COOPERATIVE DECISION MAKING DEMO ===\n")
    
    robots = create_demo_scenario()
    
    # Show initial state
    for robot in robots:
        criticality = robot.calculate_criticality(5.0)
        robot.update_criticality(criticality)
        print(f"{robot.name}: Criticality = {criticality:.3f}")
        print(f"  Energy: {[r.quantity for r in robot.R if r.type == 'energy'][0]}")
        print(f"  Health: {[r.quantity for r in robot.R if r.type == 'health'][0]}")
        print()
    
    # Demonstrate cooperative decision-making
    print("=== COOPERATIVE DECISIONS ===")
    for robot in robots:
        chosen_action = robot.cooperative_decision()
        if chosen_action:
            print(f"{robot.name} chooses: {chosen_action.name}")
        else:
            print(f"{robot.name}: No action available")
        print()

    print("END")