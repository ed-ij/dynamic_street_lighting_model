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
        self.happy = False

    def step(self):
        """
        Calculates the next position of the agent based on several factors:
        - Current Speed
        - Max Speed
        - Proximity of agent ahead of it
        - Random chance of deceleration
        """
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
            if self.model.grid.is_cell_empty(test_pos):
                distance_to_next += 1
                if distance_to_next == self.speed:
                    break
            else:
                break
        self.speed = distance_to_next

        # STEP 3: RANDOMISATION
        if self.random.random() < 0.3 and self.speed > 0:
            self.speed -= 1

        # STEP 4: MOVEMENT
        self._next_pos = self.pos
        (x, y) = self._next_pos
        x += self.speed
        self._next_pos = self.model.grid.torus_adj((x, y))

        self.model.total_speed = self.model.total_speed + self.speed

        (x, y) = self.pos
        if self.model.lighting_grid[x] == 1:
            self.happy = True
            self.model.total_happy += 1
        else:
            self.happy = False

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
        (x, y) = self.pos
        if self.historic_lit_state[0] or self.historic_lit_state[1] or self.historic_lit_state[2]:
            self.lit_state = True
            for dx in range(0, self.light_range):
                self.model.lighting_grid[x + dx] = 1
        else:
            for dx in range(0, self.light_range):
                self.model.lighting_grid[x + dx] = 0
        # print("Street Light at " + str(self.pos[0]) + " is lit? " + str(self.historic_lit_state))


class NaSchTraffic(Model):
    """
    Model class for the Nagel and Schreckenberg traffic model.
    """

    def __init__(self, height=1, width=60, vehicle_quantity=5, general_max_speed=4, seed=None):
        """"""

        super().__init__(seed=seed)
        self.height = height
        self.width = width
        self.vehicle_quantity = vehicle_quantity
        self.general_max_speed = general_max_speed
        self.schedule = SimultaneousActivation(self)
        self.grid = SingleGrid(width, height, torus=True)
        self.light_range = int(floor(30 / 2.5))

        self.average_speed = 0.0
        self.averages = []
        self.total_speed = 0
        self.lighting_grid = [0] * width
        self.total_happy = 0

        self.datacollector = DataCollector(
            model_reporters={
                "Average_Speed": "average_speed",  # Model-level count of average speed of all agents
                "Happy": "total_happy",  # Model-level count of agent happiness
            },
            # For testing purposes, agent's individual x position and speed
            # agent_reporters={
            #     "PosX": lambda x: x.pos[0],
            #     "Speed": lambda x: x.speed,
            # },
        )

        # Set up agents
        # Street lights first as these are fixed
        y = 0
        for light_iter in range(0, int(width / self.light_range)):
            x = light_iter * self.light_range
            agent = StreetLightAgent((x, y), self, self.light_range)
            self.schedule.add(agent)

        # We use a grid iterator that returns
        # the coordinates of a cell as well as
        # its contents. (coord_iter)
        cells = list(self.grid.coord_iter())
        self.random.shuffle(cells)
        for vehicle_iter in range(0, self.vehicle_quantity):
            cell = cells[vehicle_iter]
            (content, x, y) = cell
            agent = VehicleAgent((x, y), self, general_max_speed)
            self.grid.position_agent(agent, (x, y))
            self.schedule.add(agent)

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        """
        Run one step of the model. Calculate current average speed of all agents.
        """
        if self.schedule.steps == 100:
            self.running = False
        self.total_speed = 0
        self.total_happy = 0
        # Step all agents, then advance all agents
        self.schedule.step()
        if self.schedule.get_agent_count() - 5 > 0:
            self.average_speed = self.total_speed / (self.schedule.get_agent_count() - 5)
        else:
            self.average_speed = 0
        self.averages.append(self.average_speed)
        # collect data
        self.datacollector.collect(self)
