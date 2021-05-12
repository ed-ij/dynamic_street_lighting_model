import numpy as np
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

    def __init__(self, pos, model, max_speed, desired_lighting_level):
        """
        Create a new vehicle agent.
        Args:
           pos: Agent initial position in x, y.
           model: The model the agent is associated with.
           max_speed: The maximum number of cells an agent can move in a single step
        """
        super().__init__(pos, model)
        self.pos = pos
        self._next_pos = None
        self.max_speed = max_speed
        self.speed = 0
        self.desired_lighting_level = desired_lighting_level
        self.perceived_lighting = 0
        self.std_perceived_lighting = 0

    def step(self):
        """
        Calculates the next position of the agent based on several factors:
        - Current Speed
        - Max Speed
        - Proximity of agent ahead of it
        - Random chance of deceleration
        """
        # STEP 0: LOGGING
        if self.model.record_positions:
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
            if self.model.grid.is_cell_empty(test_pos):
                if self.model.lighting_feedback and self.model.lighting_grid[test_pos[0]] < self.desired_lighting_level:
                    break
                distance_to_next += 1
                if distance_to_next == self.speed:
                    break
            else:
                break
        self.speed = distance_to_next

        # STEP 3: RANDOMISATION
        if self.random.random() < self.model.p_randomisation and self.speed > 0:
            self.speed -= 1

        # LIGHTING PERCEPTION
        if self.model.lighting_model_type > 0:
            desired_visibilities = [4, 4, 4, 7, 10, 14, 19, 25]  # list of desired visibility for each speed (0-7)
            visibility = desired_visibilities[self.speed]  # set the current desired visibility
            (x, y) = self.pos
            loc_lighting = []
            max_x = len(self.model.lighting_grid)
            for dx in range(-2, visibility):  # always check for some lighting behind the vehicle
                test_x = wrap_state(x, dx, max_x)  # wrap the value of test_x to make sure it's in the valid range
                loc_lighting.append(self.model.lighting_grid[test_x])
            # average perceived lighting level is compared to desired average lighting level
            self.perceived_lighting = np.average(loc_lighting)
            self.std_perceived_lighting = np.std(loc_lighting)
        else:
            self.perceived_lighting = 0

        # STEP 4: MOVEMENT
        self._next_pos = self.pos
        (x, y) = self._next_pos
        x_next = x + self.speed
        self._next_pos = self.model.grid.torus_adj((x_next, y))

        # DATA COLLECTION
        if 0.2 * self.model.width <= x < 0.8 * self.model.width:  # agent inside measurement range
            self.model.stds_perceived_lighting.append(self.std_perceived_lighting)
            self.model.total_perceived_lighting += self.perceived_lighting
            self.model.total_speed += self.speed
            self.model.total_vehicles += 1
            if x_next >= 0.8 * self.model.width:  # agent leaving measurement range
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

    def __init__(self, pos, model, light_range, max_light_level, k):
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
        self.lit_state = 0
        self.sensor_state = 0.0
        self._decision_value = 0
        # self.historic_lit_state = deque([False, False, False], maxlen=5)
        self.max_light_level = max_light_level
        self.k = k

    def step(self):
        """
        Calculates the next next lit_state of the agent based on several factors:
        - Agents currently in sensed area
        - Neighboring lights currently lit
        - Historic lit_state
        """
        if self.model.lighting_model_type == 2:
            (x, y) = self.pos
            self.sensor_state = 0
            for dx in range(0, self.light_range):
                if not self.model.grid.is_cell_empty((x + dx, y)):
                    self.sensor_state = 1
                    break
            length = self.model.total_street_lights
            state_x = int(x / self.light_range)
            light_b1_state = self.model.lighting_grid_states[wrap_state(state_x, -1, length)]  # light -1 (before)
            light_b2_state = self.model.lighting_grid_states[wrap_state(state_x, -2, length)]  # light -2
            light_a1_state = self.model.lighting_grid_states[wrap_state(state_x, +1, length)]  # light +1 (after)
            light_a2_state = self.model.lighting_grid_states[wrap_state(state_x, +2, length)]  # light +2
            self._decision_value = 0
            self._decision_value = (self.k[0] +
                                    self.k[1] * self.lit_state +
                                    self.k[2] * self.sensor_state +
                                    self.k[3] * light_b1_state +
                                    self.k[4] * light_b2_state +
                                    self.k[5] * light_a1_state +
                                    self.k[6] * light_a2_state)
            if self.model.debug:
                print('Lit State: %.3f' % self.lit_state)
                print('Sensor State: %.3f' % self.sensor_state)
                print('Light N-1: %.3f' % light_b1_state)
                print('Light N-2: %.3f' % light_b2_state)
                print('Light N+1: %.3f' % light_a1_state)
                print('Light N+2: %.3f' % light_a2_state)
                print('Output Value: %.3f' % self._decision_value)
        else:
            self._decision_value = 1

    def advance(self):
        """
        Changes the agent lit_state to its next state.
        """
        (x, y) = self.pos
        self.model.lighting_grid_states[int(x / self.light_range)] = self.sensor_state
        if self.model.lighting_model_type == 2:
            lighting_level = clamp(100 * self._decision_value, max_n=self.max_light_level)
            self.lit_state = self._decision_value
        else:
            lighting_level = self.max_light_level
            self.lit_state = 1

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

        # calculate energy usage
        required_power = - 0.55855 * lighting_level \
                         + 0.02323 * lighting_level ** 2 \
                         - 0.00011 * lighting_level ** 3 \
                         + 17.67097

        if 0.2 * self.model.width <= x < 0.8 * self.model.width:
            self.model.total_power += required_power


class NaSchTraffic(Model):
    """
    Agent based model of traffic flow, with responsive street lighting.
    """

    def __init__(self,
                 width=200,
                 vehicle_density=0.1,
                 general_max_speed=5,
                 p_randomisation=0.5,
                 record_traffic_data=False,
                 record_positions=False,
                 max_light_level=80,
                 desired_lighting_level=30,
                 lighting_model_type=0,
                 k_parameters=None,
                 lighting_feedback=False,
                 record_lights=False,
                 debug=False,
                 seed=None):
        """"""
        self._seed = seed
        super().__init__(seed=self._seed)
        # traffic model parameters
        self.height = 1
        self.width = width
        self.vehicle_density = vehicle_density
        self.general_max_speed = general_max_speed
        self.p_randomisation = p_randomisation
        self.record_traffic_data = record_traffic_data
        self.record_positions = record_positions
        # lighting model parameters
        self.max_light_level = max_light_level  # level in the range 0-100
        # lighting model types:
        # 0 = no lighting simulated
        # 1 = constant lighting simulated
        # 2 = variable lighting simulated
        self.lighting_model_type = lighting_model_type
        # the parameters that control the variable lighting model decisions
        if k_parameters is None:
            k_parameters = (0.1, 0.3, 0.6, 0.4, 0.2, 0.2, 0.1)
        self.k_parameters = k_parameters
        # if lighting feedback is enabled the vehicles will slow down rather than entering areas that are too dark
        self.lighting_feedback = lighting_feedback
        # required lighting level for vehicles not to slow down
        self.desired_lighting_level = desired_lighting_level
        self.record_lights = record_lights
        self.debug = debug
        self.schedule = SimultaneousActivation(self)
        self.grid = SingleGrid(self.width, self.height, torus=True)
        self.light_range = int(floor(36 / 4.5))
        self.lighting_grid = [20] * width
        self.agent_position_log = []
        self.light_value_log = []

        self.total_street_lights = 0
        self.total_speed = 0
        self.total_perceived_lighting = 0
        self.stds_perceived_lighting = []
        self.total_vehicles = 0
        self.total_flow = 0
        self.total_power = 0

        self.average_speed = 0.0
        self.average_perceived_lighting = 0.0
        self.max_std_perceived_lighting = 0.0
        self.current_density = 0.0
        self.average_lighting_level = 0.0
        self.average_power = 0.0

        # batch runner data collection arrays
        self.speed_averages = []
        self.perceived_lighting_averages = []
        self.max_stds_perceived_lighting = []
        self.densities = []
        self.flows = []
        self.lighting_averages = []
        self.power_averages = []

        model_reporters = {}
        # For testing purposes, agent's individual x position and speed
        agent_reporters = {}
        #     "PosX": lambda x: x.pos[0],
        #     "Speed": lambda x: x.speed,
        # }
        if self.record_traffic_data:
            model_reporters['Average_Speed'] = 'average_speed'  # Model-level count of average speed of all agents
            model_reporters['Density'] = 'current_density'
            model_reporters['Flow'] = 'total_flow'

        if self.record_positions:
            model_reporters['Agent_Positions'] = 'agent_position_log'

        if self.lighting_model_type > 0:
            model_reporters['Average_Perceived_Lighting'] = 'average_perceived_lighting'  # Model-level count of agent perceived lighting levels
            model_reporters['Lighting_Level'] = 'average_lighting_level'
            model_reporters['Average_Power'] = 'average_power'
            model_reporters['Max_Standard_Deviation_Lighting'] = 'max_std_perceived_lighting'
            if self.record_lights:
                model_reporters['Lighting_Values'] = 'light_value_log'

        self.datacollector = DataCollector(model_reporters, agent_reporters)

        # Set up agents

        # Street lights first as these are fixed
        if self.lighting_model_type > 0:
            y = 0
            for light_iter in range(0, int(width / self.light_range)):
                x = light_iter * self.light_range
                agent = StreetLightAgent((x, y),
                                         self,
                                         self.light_range,
                                         self.max_light_level,
                                         self.k_parameters)
                self.schedule.add(agent)
                self.total_street_lights += 1

            self.lighting_grid_states = [0] * self.total_street_lights

            if self.debug:
                print("Added " + str(self.total_street_lights) + " lights")

        # We use a grid iterator that returns the coordinates of a
        # cell as well as its contents. (coord_iter)
        cells = list(self.grid.coord_iter())
        self.random.shuffle(cells)
        vehicle_quantity = int(width * self.vehicle_density)
        for vehicle_iter in range(0, vehicle_quantity):
            cell = cells[vehicle_iter]
            (content, x, y) = cell
            agent = VehicleAgent((x, y),
                                 self,
                                 general_max_speed,
                                 self.desired_lighting_level)
            self.grid.position_agent(agent, (x, y))
            self.schedule.add(agent)

        if self.debug:
            print("Added " + str(vehicle_quantity) + " vehicles")

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        """
        Run one step of the model. Calculate current average speed of all agents.
        """

        self.total_speed = 0
        self.total_perceived_lighting = 0
        self.total_vehicles = 0
        self.total_flow = 0
        self.total_power = 0
        self.agent_position_log = []
        self.light_value_log = []
        self.stds_perceived_lighting = []

        # Step all agents, then advance all agents
        self.schedule.step()

        if self.total_vehicles > 0:
            self.average_speed = self.total_speed / self.total_vehicles
            self.average_perceived_lighting = self.total_perceived_lighting / self.total_vehicles
            self.max_std_perceived_lighting = np.max(self.stds_perceived_lighting)
            self.current_density = self.total_vehicles / (self.width * 0.6)
        else:
            self.average_speed = 0
            self.average_perceived_lighting = 0
            self.current_density = 0
            self.max_std_perceived_lighting = 0

        if self.total_power > 0:
            self.average_power = self.total_power / (self.total_street_lights*0.6)
        else:
            self.average_power = 0.0

        # data collection for batch runner
        if self.record_traffic_data:
            self.speed_averages.append(self.average_speed)
            self.densities.append(self.current_density)
            self.flows.append(float(self.total_flow))

        if self.lighting_model_type > 0:
            lighting_subset = self.lighting_grid[int(self.width * 0.2):int(self.width * 0.8)]
            self.average_lighting_level = sum(lighting_subset) / len(lighting_subset)
            self.lighting_averages.append(self.average_lighting_level)
            self.perceived_lighting_averages.append(self.average_perceived_lighting)
            self.max_stds_perceived_lighting.append(self.max_std_perceived_lighting)
            self.power_averages.append(self.average_power)
            if self.record_lights:
                self.light_value_log = lighting_subset

        # data collection for single runs
        self.datacollector.collect(self)


def wrap_state(x, dx, max_x):
    new_x = x + dx
    if new_x < 0:
        new_x = max_x + new_x  # new_x is negative so: max_x + (-x) gives new position near end of array
    elif new_x >= max_x:
        new_x = new_x - max_x  # new_x is larger than max_x so wrapping around to small x values
    return new_x


def clamp(n, max_n, min_n=0):
    return max(min(max_n, n), min_n)
