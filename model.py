from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import SingleGrid
from mesa.datacollection import DataCollector


class VehicleAgent(Agent):
    """
    Vehicle agent
    """

    def __init__(self, pos, model, maxspeed):
        """
        Create a new vehicle agent.
        Args:
           pos: Agent initial position in x, y.
           model: The model the agent is associated with.
           maxspeed: The maximum number of cells an agent can move in a single step
        """
        super().__init__(pos, model)
        self.pos = pos
        self.speed = 0
        self.maxspeed = maxspeed
        self._nextPos = None

    def step(self):
        """
        Calculates the next position of the agent based on several factors:
        - Current Speed
        - Max Speed
        - Proximity of agent ahead of it
        - Random chance of deceleration
        """
        # STEP 1: ACCELERATION
        if self.speed < self.maxspeed:
            self.speed += 1

        # STEP 2: DECELERATION
        distancetonext = 0
        for distance in range(self.maxspeed):
            distance += 1
            testposlist = list(self.pos)
            testposlist[0] = testposlist[0] + distance
            testpos = self.model.grid.torus_adj(tuple(testposlist))
            if self.model.grid.is_cell_empty(testpos):
                distancetonext += 1
            else:
                break
        if distancetonext < self.speed:
            self.speed = distancetonext

        # STEP 3: RANDOMISATION
        if self.random.random() < 0.3 and self.speed > 0:
            self.speed -= 1

        # STEP 4: MOVEMENT
        self._nextPos = self.pos
        temppos = list(self._nextPos)
        temppos[0] = temppos[0] + self.speed
        self._nextPos = self.model.grid.torus_adj(tuple(temppos))

        self.model.totalspeed = self.model.totalspeed + self.speed

    def advance(self):
        """
        Moves the agent to its next position.
        """
        self.model.grid.move_agent(self, self._nextPos)


class NaSchTraffic(Model):
    """
    Model class for the Nagel and Schreckenberg traffic model.
    """

    def __init__(self, height=1, width=60, density=0.2, generalmaxspeed=4, seed=None):
        """"""

        super().__init__(seed=seed)
        self.height = height
        self.width = width
        self.density = density
        self.generalmaxspeed = generalmaxspeed
        self.schedule = SimultaneousActivation(self)
        self.grid = SingleGrid(width, height, torus=True)

        self.averagespeed = 0.0
        self.averages = []
        self.totalspeed = 0

        self.datacollector = DataCollector(
            model_reporters={"AverageSpeed": "averagespeed"},  # Model-level count of average speed of all agents
            # For testing purposes, agent's individual x position and speed
            agent_reporters={
                "PosX": lambda x: x.pos[0],
                "Speed": lambda x: x.speed,
            },
        )

        # Set up agents
        # We use a grid iterator that returns
        # the coordinates of a cell as well as
        # its contents. (coord_iter)
        for cell in self.grid.coord_iter():
            x = cell[1]
            y = cell[2]
            if self.random.random() < self.density:
                agent = VehicleAgent((x, y), self, generalmaxspeed)
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
        self.totalspeed = 0
        # Step all agents, then advance all agents
        self.schedule.step()
        if self.schedule.get_agent_count() > 0:
            self.averagespeed = self.totalspeed / self.schedule.get_agent_count()
        else:
            self.averagespeed = 0
        self.averages.append(self.averagespeed)
        # collect data
        self.datacollector.collect(self)
