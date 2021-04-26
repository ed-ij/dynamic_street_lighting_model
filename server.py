from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import CanvasGrid, ChartModule, TextElement
from mesa.visualization.UserParam import UserSettableParameter

from model import NaSchTraffic

# Define some colours for later use
# Red
STOPPED_COLOR = ["#FF0000", "#FF9999"]
# Orange
SLOW_COLOR = ["#FFAB00", "#FFCB62"]
# Green
FAST_COLOR = ["#2BCB00", "#8EE484"]
# Black
UNHAPPY_COLOR = ["#000000", "#4D4D4D"]


class AgentElement(TextElement):
    """
    Display text for the average speed of the agents.
    """

    def __init__(self):
        pass

    def render(self, model):
        text_1 = "Number of agents: " + str(model.schedule.get_agent_count()-model.total_street_lights) + "   "
        text_3 = ("Average lighting level at "
                  + str(int(sum(model.lighting_grid)/(model.light_range*model.total_street_lights)))
                  + "%")
        return text_1 + text_3


def vehicle_draw(agent):
    """
    Portrayal Method for canvas
    """
    if agent is None:
        return
    portrayal = {"Shape": "circle", "r": 1, "Filled": "true", "Layer": 1, }

    if not agent.happy:
        color = UNHAPPY_COLOR
    elif agent.speed == 0:
        color = STOPPED_COLOR
    elif agent.speed > int(agent.max_speed/2):
        color = FAST_COLOR
    else:
        color = SLOW_COLOR
    portrayal["Color"] = color

    return portrayal

# define the elements of the visualisation
agent_element = AgentElement()
canvas_element = CanvasGrid(vehicle_draw, 200, 1, 1300, 20)
speed_chart = ChartModule([{"Label": "Average_Speed", "Color": "Black"}])
density_chart = ChartModule([{"Label": "Density", "Color": "Black"}])
lighting_chart = ChartModule([{"Label": "Lighting_Level", "Color": "Blue"}])
happy_chart = ChartModule([{"Label": "Average_Happiness", "Color": "Blue"}])

# define the parameters of the model
model_params = {
    "height": 1,
    "width": 200,
    "debug": 2,
    "vehicle_density": UserSettableParameter("slider", "Vehicle Density", 0.1, 0.01, 0.3, 0.01),
    "general_max_speed": UserSettableParameter("slider", "Max speed", 5, 3, 6, 1),
    "p_randomisation": UserSettableParameter("slider", "Probability of Randomisation", 0.4, 0.05, 0.95, 0.05)
}

# instantiate server
server = ModularServer(
    NaSchTraffic, [canvas_element, agent_element, speed_chart, density_chart, lighting_chart, happy_chart], "Traffic Model", model_params
)
# set server port
server.port = 8555
