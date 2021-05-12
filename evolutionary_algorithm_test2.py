import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time
import seaborn as sns

import random
import pickle
from numpy.random import default_rng
from deap import base
from deap import creator
from deap import tools

from model import NaSchTraffic
from mesa.batchrunner import BatchRunner

# from dask.distributed import Client
# client = Client(processes=False, )
from scoop import futures

sns.set_theme()
plt.rcParams['figure.figsize'] = [6, 4]
plt.rcParams['figure.dpi'] = 300
rng = default_rng()

# Shared parameters
test_width = 200
test_density = 0.01
test_max_speed = 5
test_k_params = (0.3,  # k0: Offset
                 0.2,  # k1: Lit state
                 0.6,  # k2: Sensor state
                 0.5,  # k3: Light N-1
                 0.2,  # k4: Light N-2
                 0.1,  # k5: Light N+1
                 0.0)  # k6: Light N+2

test_length = 1000
test_iters = 15


def get_average_power(model):
    '''
    Find the average power over the steps after equilibrium is reached.
    '''
    return np.average(model.power_averages[int(test_length / 5):])


def get_average_perceived_lighting(model):
    '''
    Find the average perceived lighting over the steps after equilibrium is reached.
    '''

    return np.average(model.perceived_lighting_averages[int(test_length / 5):])


def get_max_std_perceived_lighting(model):
    '''
    Find the max standard deviation of the perceived lighting over the steps after equilibrium is reached.
    '''

    return np.max(model.max_stds_perceived_lighting[int(test_length / 5):])


model_reporters = {
    'Averaged_Power': get_average_power,
    'Averaged_Perceived_Lighting': get_average_perceived_lighting,
    'Max_STD_Perceived_Lighting': get_max_std_perceived_lighting,
}

IND_SIZE = 7

creator.create('FitnessCombined', base.Fitness, weights=(-1.0, -1.0))
creator.create('Individual', list, fitness=creator.FitnessCombined)

toolbox = base.Toolbox()
# Attribute generator
toolbox.register('attr_float', random.random)
# Structure initializers
toolbox.register('individual', tools.initRepeat, creator.Individual,
                 toolbox.attr_float, n=IND_SIZE)
toolbox.register('population', tools.initRepeat, list, toolbox.individual)


def evaluate_ind(individual):
    l_fixed_params = {'width': test_width,
                      'vehicle_density': test_density,
                      'general_max_speed': test_max_speed,
                      'lighting_model_type': 2,
                      'k_parameters': tuple(individual),
                      }

    l_param_sweep = BatchRunner(NaSchTraffic,
                                fixed_parameters=l_fixed_params,
                                iterations=test_iters,
                                max_steps=test_length,
                                model_reporters=model_reporters,
                                display_progress=False)

    l_param_sweep.run_all()
    l_df = l_param_sweep.get_model_vars_dataframe().sort_values('Averaged_Power')

    perceived_lighting = np.average(l_df.Averaged_Perceived_Lighting)
    perceived_lighting_error = max(45 - perceived_lighting, 0)

    # np.average(l_df.Max_STD_Perceived_Lighting)

    return (perceived_lighting_error,
            np.average(l_df.Averaged_Power))


N_BEST, N_VAR, N_GEN, FREQ = 5, 15, 80, 1
STARTING_POP = 200

toolbox.register('mutate', tools.mutGaussian, mu=0, sigma=1, indpb=0.08)
toolbox.register('select', tools.selBest, k=N_BEST)
toolbox.register('evaluate', evaluate_ind)


# noinspection DuplicatedCode
def main(checkpoint=None):
    toolbox.register('map', futures.map)
    time_str = time.strftime('%y/%m/%d %H:%M:%S')
    print(time_str)
    if checkpoint:
        print('loading starting data from file')
        # A file name has been given, then load the data from the file
        with open(checkpoint, 'rb') as cp_file:
            cp = pickle.load(cp_file)
        pop = cp['population']
        start_gen = cp['generation']
        halloffame = cp['halloffame']
        logbook = cp['logbook']
        random.setstate(cp['rndstate'])
    else:
        pop = toolbox.population(n=STARTING_POP)
        start_gen = 0
        halloffame = tools.HallOfFame(maxsize=15)
        logbook = tools.Logbook()

        print('Generating fitness values for starting population of %i individuals' % STARTING_POP)

    stats = tools.Statistics(key=lambda ind: ind.fitness.values)
    stats.register('avg', np.mean, axis=0)
    stats.register('std', np.std, axis=0)

    invalid_ind = [ind for ind in pop if not ind.fitness.valid]
    fitnesses = list(toolbox.map(toolbox.evaluate, invalid_ind))
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    for g in range(start_gen, N_GEN):
        str_1 = ('-- Generation %i -- ' % g)
        time_str = time.strftime('%y/%m/%d %H:%M:%S')
        print(str_1, time_str)
        # log best results in each generation
        best_ind = [ind for ind in toolbox.select(pop, k=1)]
        best_fitness = [ind.fitness.values for ind in toolbox.select(pop, k=1)]
        # store overall best results
        halloffame.update(pop)
        # record stats to logbook
        record = stats.compile(pop)
        logbook.record(gen=g, evals=len(pop), best_i=best_ind, best_v=best_fitness, **record)
        print(logbook.stream)
        # Select the best performing individuals
        selected = toolbox.select(pop)
        # Clone the selected individuals
        selected = list(toolbox.map(toolbox.clone, selected))
        pop[:] = selected
        # Creat N_VAR copies of the selected individuals
        offspring = []
        for _ in range(N_VAR):
            offspring.extend(list(toolbox.map(toolbox.clone, selected)))
        # mutate children and remove fitness values
        for child in offspring:
            toolbox.mutate(child)
            del child.fitness.values

        # add some additional random agents
        additional_spawn = toolbox.population(n=N_VAR)
        offspring.extend(additional_spawn)

        fitnesses = list(toolbox.map(toolbox.evaluate, offspring))
        for ind, fit in zip(offspring, fitnesses):
            ind.fitness.values = fit

        pop.extend(offspring)

        if g % FREQ == 0:
            # Fill the dictionary using the dict(key=value[, ...]) constructor
            cp = dict(population=pop, generation=g, halloffame=halloffame,
                      logbook=logbook, rndstate=random.getstate())

            with open('data/evolution_long_run_21.pkl', 'wb') as cp_file:
                pickle.dump(cp, cp_file)

    gen = logbook.select('gen')
    logged_best = [ind[0] for ind in logbook.select('best_v')]
    logged_avgs = logbook.select('avg')

    fig, ax1 = plt.subplots()
    line1 = ax1.plot(gen, logged_best, '#4c72b0', label='Best Fitness')
    line2 = ax1.plot(gen, logged_avgs, '#c44e52', label='Average Fitness')
    ax1.set_xlabel('Generation')
    ax1.set_ylabel('Fitness', color='b')

    lns = line1 + line2
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc='center right')

    plt.savefig('img/genetic_tests_21.png')


if __name__ == '__main__':
    main()
    # 'evolution_long_run_21.pkl'
