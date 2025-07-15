import mesa
import mesa.visualization as vis
from robot_base import RobotSimulation, RobotAgent, BoxAgent, NestAgent


def agent_portrayal(agent):
    """Define how agents appear in the visualization"""

    if isinstance(agent, RobotAgent):
        if not agent.is_alive:
            # Dead robot
            portrayal = {"Shape": "circle", "Filled": "true", "Color": "black", "r": 0.3}
        else:
            # Alive robot - color based on robot color and battery level
            colors = {"red": "#FF0000", "green": "#00FF00", "blue": "#0000FF"}
            base_color = colors.get(agent.color, "#808080")

            # Adjust opacity based on battery level
            battery_ratio = agent.battery / agent.max_battery
            opacity = max(0.3, battery_ratio)  # Never fully transparent

            portrayal = {
                "Shape": "circle",
                "Filled": "true",
                "Color": base_color,
                "r": 0.8,
                "opacity": opacity
            }

            # Show if robot is carrying a box
            if agent.carried_box:
                portrayal["Shape"] = "rect"
                portrayal["w"] = 0.8
                portrayal["h"] = 0.8

        # Add text showing battery level
        portrayal["text"] = f"{agent.battery}"
        portrayal["text_color"] = "white"

    elif isinstance(agent, BoxAgent):
        if not agent.is_carried:
            colors = {"red": "#FF4444", "green": "#44FF44", "blue": "#4444FF"}
            portrayal = {
                "Shape": "rect",
                "Filled": "true",
                "Color": colors.get(agent.color, "#CCCCCC"),
                "w": 0.6,
                "h": 0.6
            }
        else:
            # Don't show carried boxes (they move with robots)
            portrayal = {"Shape": "circle", "Filled": "false", "r": 0}

    elif isinstance(agent, NestAgent):
        colors = {"red": "#880000", "green": "#008800", "blue": "#000088"}
        portrayal = {
            "Shape": "rect",
            "Filled": "true",
            "Color": colors.get(agent.color, "#444444"),
            "w": 1.5,
            "h": 1.5,
            "text": f"Nest\n{agent.deposited_boxes}",
            "text_color": "white"
        }

    else:
        # Default portrayal
        portrayal = {"Shape": "circle", "Filled": "true", "Color": "gray", "r": 0.2}

    return portrayal


def create_server():
    """Create Mesa visualization server"""

    # Grid visualization
    grid = vis.CanvasGrid(agent_portrayal, 50, 50, 800, 800)

    # Charts
    battery_chart = vis.ChartModule([
        {"Label": "Mean_Battery", "Color": "blue"}
    ], data_collector_name='datacollector')

    alive_chart = vis.ChartModule([
        {"Label": "Alive_Robots", "Color": "green"}
    ], data_collector_name='datacollector')

    boxes_chart = vis.ChartModule([
        {"Label": "Total_Boxes", "Color": "red"},
        {"Label": "Red_Delivered", "Color": "darkred"},
        {"Label": "Green_Delivered", "Color": "darkgreen"},
        {"Label": "Blue_Delivered", "Color": "darkblue"}
    ], data_collector_name='datacollector')

    # Model parameters for the interface
    model_params = {
        "robot_type": vis.Choice(
            "Robot Type",
            value="cooperative",
            choices=["random", "greedy", "cooperative", "saphesia"]
        ),
        "n_robots_per_color": vis.Slider(
            "Robots per Color",
            value=30,
            min_val=10,
            max_val=50,
            step=5
        ),
        "width": vis.Slider("Grid Width", value=50, min_val=30, max_val=100, step=5),
        "height": vis.Slider("Grid Height", value=50, min_val=30, max_val=100, step=5)
    }

    # Create server
    server = vis.ModularServer(
        RobotSimulation,
        [grid, battery_chart, alive_chart, boxes_chart],
        "Robot Cooperation Simulation",
        model_params
    )

    return server


if __name__ == "__main__":
    # Create and launch the visualization server
    server = create_server()
    server.port = 8521  # Default Mesa port

    print("Starting Mesa visualization server...")
    print("Open your browser and go to: http://localhost:8521")
    print("\nVisualization Legend:")
    print("- Circles: Robots (opacity = battery level)")
    print("- Small squares: Boxes")
    print("- Large squares: Nests")
    print("- Black circles: Dead robots")
    print("- Rectangles: Robots carrying boxes")

    server.launch()