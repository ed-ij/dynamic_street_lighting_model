from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import SingleGrid
from mesa.datacollection import DataCollector
from collections import deque
from math import floor


class VehicleAgent(Agent):
    """
    Vehicle agent
    """

    def __init__(self, pos, model, max_speed):
        """
        Create a new vehicle agent.
        Args:
           pos: Agent initial position in x, y.
           model: The model the agent is associated with.
           max_speed: The maximum number of cells an agent can move in a single step
        """
        super().__init__(pos, model)
        self.pos = pos
        self.speed = 0
        self.max_speed = max_speed
        self._next_pos = None
        self.happy = 0

    def step(self):
        """
        Calculates the next position of the agent based on several factors:
        - Current Speed
        - Max Speed
        - Proximity of agent ahead of it
        - Random chance of deceleration
        """
        # STEP 0: LOGGING
        if self.model.debug == 1 or self.model.debug == 3:
            log_entry = (self.pos[0], self.model.schedule.steps, self.speed)
            self.model.agent_position_log.append(log_entry)

        # STEP 1: ACCELERATION
        if self.speed < self.max_speed:
            self.speed += 1

        # STEP 2: DECELERATION
        distance_to_next = 0
        (x, y) = self.pos
        for distance in range(self.max_speed):
            distance += 1
            test_x = x + distance
            test_pos = self.model.grid.torus_adj((test_x, y))
            if self.model.grid.is_cell_empty(test_pos): #and self.model.lighting_grid[test_pos[0]] > 50: #this needs lighting sensors implemented to work
                distance_to_next += 1
                if distance_to_next == self.speed:
                    break
            else:
                break
        self.speed = distance_to_next

        # STEP 3: RANDOMISATION
        if self.random.random() < self.model.p_randomisation and self.speed > 0:
            self.speed -= 1

        # HAPPINESS
        desired_visibilities = [3, 3, 3, 6, 9, 13, 18, 24]
        visibility = desired_visibilities[self.speed]
        (x, y) = self.pos
        loc_lighting = 0
        max_x = len(self.model.lighting_grid)
        for dx in range(-2, visibility+1):
            test_x = x + dx
            if test_x >= max_x:
                test_x = test_x - max_x
            elif test_x < 0:
                test_x = max_x + test_x     # test_x is negative so add rather than subtract
            loc_lighting += self.model.lighting_grid[test_x]
        loc_happy = (loc_lighting/(visibility+3))/70
        if loc_happy > 1:
            self.happy = 1
        else:
            self.happy = loc_happy

        # STEP 4: MOVEMENT
        self._next_pos = self.pos
        (x, y) = self._next_pos
        x_next = x + self.speed
        self._next_pos = self.model.grid.torus_adj((x_next, y))

        # DATA COLLECTION
        if 0.2*self.model.width <= x < 0.8*self.model.width:         # agent inside measurement range
            self.model.total_happy += self.happy
            self.model.total_speed = self.model.total_speed + self.speed
            self.model.total_vehicles += 1
            if x_next >= 0.8*self.model.width:           # agent leaving measurement range
                self.model.total_flow += 1

    def advance(self):
        """
        Moves the agent to its next position.
        """
        self.model.grid.move_agent(self, self._next_pos)


class StreetLightAgent(Agent):
    """
    Street light agent
    """

    def __init__(self, pos, model, light_range):
        """
        Create a new street light agent.
        Args:
           pos: Agent initial position in x, y.
           model: The model the agent is associated with.
           light_range: The maximum number of cells the street light can sense/light up
        """
        super().__init__(10000 + pos[0], model)
        self.pos = pos
        self.light_range = light_range
        self.lit_state = False
        self.historic_lit_state = deque([False, False, False, False, False], maxlen=5)

    def step(self):
        """
        Calculates the next next lit_state of the agent based on several factors:
        - Agents currently in sensed area
        - Neighboring lights currently lit
        - Historic lit_state
        """
        temp_lit_state = False
        (x, y) = self.pos
        for dx in range(0, self.light_range):
            if not self.model.grid.is_cell_empty((x+dx, y)):
                temp_lit_state = True
                break
        self.historic_lit_state.appendleft(temp_lit_state)

    def advance(self):
        """
        Changes the agent lit_state to its next state.
        """
        if self.historic_lit_state[0]:
            self.lit_state = True
            lighting_level = 80
        elif self.historic_lit_state[1]:
            self.lit_state = True
            lighting_level = 50
        elif self.historic_lit_state[2]:
            self.lit_state = True
            lighting_level = 35
        else:
            lighting_level = 20

        (x, y) = self.pos
        for dx in range(0, self.light_range):
            if dx == 0 or dx == 7:
                adjusted_level = lighting_level * 0.40
            elif dx == 1 or dx == 6:
                adjusted_level = lighting_level * 0.52
            elif dx == 2 or dx == 5:
                adjusted_level = lighting_level * 0.71
            elif dx == 3 or dx == 4:
                adjusted_level = lighting_level
            self.model.lighting_grid[x + dx] = adjusted_level
        # print("Street Light at " + str(self.pos[0]) + " is lit? " + str(self.historic_lit_state))


class NaSchTraffic(Model):
    """
    Agent based model of traffic flow, with responsive street lighting. Happiness is measured by the level of lighting
    in cells occupied by, and ahead of, agents.
    """

    def __init__(self,
                 height=1,
                 width=200,
                 vehicle_density=0.1,
                 general_max_speed=5,
                 p_randomisation=0.4,
                 debug=0,
                 seed=None):
        """"""

        super().__init__(seed=seed)
        self.height = height
        self.width = width
        self.vehicle_density = vehicle_density
        self.general_max_speed = general_max_speed
        self.p_randomisation = p_randomisation
        self.debug = debug
        self.schedule = SimultaneousActivation(self)
        self.grid = SingleGrid(width, height, torus=True)
        self.light_range = int(floor(36 / 4.5))
        self.lighting_grid = [20] * width
        self.agent_position_log = []

        self.total_street_lights = 0
        self.total_speed = 0
        self.total_happy = 0
        self.total_vehicles = 0
        self.total_flow = 0

        self.average_speed = 0.0
        self.average_happy = 0.0
        self.current_density = 0.0
        self.average_lighting_level = 0.0

        self.speed_averages = []
        self.happiness_averages = []
        self.densities = []
        self.flows = []
        self.lighting_averages = []

        if self.debug == 1 or self.debug == 3:
            self.datacollector = DataCollector(
                model_reporters={
                    "Average_Speed": "average_speed",  # Model-level count of average speed of all agents
                    # "Average_Happiness": "average_happy",  # Model-level count of agent happiness
                    "Density": "current_density",
                    "Flow": "total_flow",
                    # "Lighting_Level": "average_lighting_level",
                    "Agent_Positions": "agent_position_log",
                },
                # For testing purposes, agent's individual x position and speed
                # agent_reporters={
                #     "PosX": lambda x: x.pos[0],
                #     "Speed": lambda x: x.speed,
                # },
            )
        else:
            self.datacollector = DataCollector(
                model_reporters={
                    "Average_Speed": "average_speed",  # Model-level count of average speed of all agents
                    # "Average_Happiness": "average_happy",  # Model-level count of agent happiness
                    "Density": "current_density",
                    "Flow": "total_flow",
                    # "Lighting_Level": "average_lighting_level",
                },
            )

        # Set up agents
        # Street lights first as these are fixed
        y = 0
        for light_iter in range(0, int(width / self.light_range)):
            x = light_iter * self.light_range
            agent = StreetLightAgent((x, y), self, self.light_range)
            self.schedule.add(agent)
            self.total_street_lights += 1

        if self.debug > 1:
            print("Added " + str(self.total_street_lights) + " lights")

        # We use a grid iterator that returns
        # the coordinates of a cell as well as
        # its contents. (coord_iter)
        cells = list(self.grid.coord_iter())
        self.random.shuffle(cells)
        vehicle_quantity = int(width*self.vehicle_density)
        for vehicle_iter in range(0, vehicle_quantity):
            cell = cells[vehicle_iter]
            (content, x, y) = cell
            agent = VehicleAgent((x, y), self, general_max_speed)
            self.grid.position_agent(agent, (x, y))
            self.schedule.add(agent)

        if self.debug > 1:
            print("Added " + str(vehicle_quantity) + " vehicles")

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        """
        Run one step of the model. Calculate current average speed of all agents.
        """

        self.total_speed = 0
        self.total_happy = 0
        self.total_vehicles = 0
        self.total_flow = 0
        self.agent_position_log = []
        # Step all agents, then advance all agents
        self.schedule.step()
        if self.total_vehicles > 0:
            self.average_speed = self.total_speed / self.total_vehicles
            self.average_happy = self.total_happy / self.total_vehicles
            self.current_density = self.total_vehicles / (self.width*0.6)
        else:
            self.average_speed = 0
            self.average_happy = 0
            self.current_density = 0

        lighting_subset = self.lighting_grid[int(self.width*0.2):int(self.width*0.8)]
        self.average_lighting_level = sum(lighting_subset) / len(lighting_subset)

        self.speed_averages.append(self.average_speed)
        # self.happiness_averages.append(self.average_happy)
        self.densities.append(self.current_density)
        self.flows.append(float(self.total_flow))
        # self.lighting_averages.append(self.average_lighting_level)

        # collect data
        self.datacollector.collect(self)
