from component_system import ComponentSystem
from resource import Resource
from goals import GoalResource
from functionality import Functionality

def create_enhanced_demo():
    """Create a more complex demo with diverse functionalities"""
    
    # Create systems with different specializations
    medic = ComponentSystem("Medic_Bot")
    scout = ComponentSystem("Scout_Bot") 
    support = ComponentSystem("Support_Bot")
    
    # Different resource states to create interesting dynamics
    medic.add_resource(Resource("energy", 85))
    medic.add_resource(Resource("health", 45))
    medic.add_resource(Resource("repair_kits", 3))
    
    scout.add_resource(Resource("energy", 20))
    scout.add_resource(Resource("health", 90))
    scout.add_resource(Resource("intel", 0))
    
    support.add_resource(Resource("energy", 95))
    support.add_resource(Resource("health", 70))
    support.add_resource(Resource("supplies", 5))
    
    # Add goals with different priorities
    medic.add_goal_resource(GoalResource("energy", 80, "EQ", 1))
    medic.add_goal_resource(GoalResource("health", 90, "EQ", 3))  # High priority
    medic.add_goal_resource(GoalResource("repair_kits", 2, "EQ", 2))
    
    scout.add_goal_resource(GoalResource("energy", 80, "EQ", 3))  # High priority
    scout.add_goal_resource(GoalResource("health", 85, "EQ", 1))
    scout.add_goal_resource(GoalResource("intel", 10, "EQ", 2))
    
    support.add_goal_resource(GoalResource("energy", 90, "EQ", 1))
    support.add_goal_resource(GoalResource("health", 80, "EQ", 2))
    support.add_goal_resource(GoalResource("supplies", 3, "EQ", 2))
    
    # Create specialized functionalities
    functionalities = {
        "heal_others": Functionality("heal_others", 3.0, {"health": 20, "repair_kits": -1}),
        "gather_intel": Functionality("gather_intel", 2.0, {"intel": 5, "energy": -15}),
        "share_energy": Functionality("share_energy", 1.0, {"energy": 15}),
        "repair_self": Functionality("repair_self", 4.0, {"health": 10, "energy": -5}),
        "resupply": Functionality("resupply", 2.0, {"supplies": 2, "energy": -10}),
        "emergency_rest": Functionality("emergency_rest", 6.0, {"energy": 30, "health": 5})
    }
    
    # Assign functionalities based on specialization
    for f_name in ["heal_others", "share_energy", "repair_self", "emergency_rest"]:
        medic.add_functionality(functionalities[f_name])
    
    for f_name in ["gather_intel", "share_energy", "repair_self", "emergency_rest"]:
        scout.add_functionality(functionalities[f_name])
    
    for f_name in ["resupply", "share_energy", "heal_others", "emergency_rest"]:
        support.add_functionality(functionalities[f_name])
    
    # Create network topology
    medic.add_link(scout)
    medic.add_link(support)
    scout.add_link(medic)
    scout.add_link(support)
    support.add_link(medic)
    support.add_link(scout)
    
    return [medic, scout, support]

def run_cooperative_simulation(robots, steps=3):
    """Run multi-step simulation to see cooperative behavior evolve"""
    
    for step in range(steps):
        print(f"\n{'='*20} STEP {step + 1} {'='*20}")
        
        # Show current state
        print("Current State:")
        for robot in robots:
            if step == 0:
                criticality = robot.calculate_criticality(3.0)
                robot.update_criticality(criticality)
            else:
                criticality = robot.get_criticality()
            
            print(f"  {robot.name}: Criticality = {criticality:.3f}")
            for resource in robot.R:
                print(f"    {resource.type}: {resource.quantity}")
        
        # Make cooperative decisions
        print("\nCooperative Decisions:")
        decisions = {}
        for robot in robots:
            chosen_action = robot.cooperative_decision_debug()
            decisions[robot] = chosen_action
            if chosen_action:
                print(f"  {robot.name} chooses: {chosen_action.name}")
                # Show reasoning
                print(f"    Duration: {chosen_action.duration}, Effects: {chosen_action.resource_effects}")
            else:
                print(f"  {robot.name}: No action available")
        
        # Simulate effects (simplified)
        print("\nSimulating Effects...")
        for robot, action in decisions.items():
            if action:
                # Apply effects to robot's resources
                for resource_type, change in action.resource_effects.items():
                    for resource in robot.R:
                        if resource.type == resource_type:
                            old_qty = resource.quantity
                            resource.quantity = max(0, resource.quantity + change)
                            print(f"  {robot.name}: {resource_type} {old_qty} -> {resource.quantity}")
                
                # Update criticality
                new_criticality = robot.calculate_criticality(3.0)
                robot.update_criticality(new_criticality)
    

    # Show end state
    print("end State:")
    for robot in robots:
        criticality = robot.get_criticality()
        print(f"  {robot.name}: Criticality = {criticality:.3f}")
        for resource in robot.R:
            print(f"    {resource.type}: {resource.quantity}")

if __name__ == "__main__":
    print("=== ENHANCED COOPERATIVE DECISION MAKING DEMO ===")
    
    robots = create_enhanced_demo()
    run_cooperative_simulation(robots, steps=2)