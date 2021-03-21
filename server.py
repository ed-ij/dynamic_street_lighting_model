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
        text_1 = "Number of agents: " + str(model.schedule.get_agent_count()-5) + "   "
        text_2 = "Happiness: " + str(model.totalhappy) + "   "
        text_3 = "Lit Lights: " + str(int(sum(model.lighting_grid)/12)) + " out of 5   "
        return text_1 + text_2 + text_3


def vehicle_draw(agent):
    """
    Portrayal Method for canvas
    """
    if agent is None:
        return
    portrayal = {"Shape": "circle", "r": 0.5, "Filled": "true", "Layer": 0, }

    if not agent.happy:
        color = UNHAPPY_COLOR
    elif agent.speed == 0:
        color = STOPPED_COLOR
    elif agent.speed > int(agent.maxspeed/2):
        color = FAST_COLOR
    else:
        color = SLOW_COLOR
    portrayal["Color"] = color

    return portrayal

# define the elements of the visualisation
agent_element = AgentElement()
canvas_element = CanvasGrid(vehicle_draw, 60, 1, 1000, 20)
speed_chart = ChartModule([{"Label": "AverageSpeed", "Color": "Black"}])

# define the parameters of the model
model_params = {
    "height": 1,
    "width": 60,
    "vehicle_quantity": UserSettableParameter("slider", "Vehicle Quantity", 5, 1, 30, 1),
    "generalmaxspeed": UserSettableParameter("slider", "Max speed", 4, 1, 6, 1),
}

# instantiate server
server = ModularServer(
    NaSchTraffic, [canvas_element, agent_element, speed_chart], "Traffic Model", model_params
)
# set server port
server.port = 8555
