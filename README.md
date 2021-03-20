# Agent-Based Traffic Model

## Summary

This model is a looped implementation of the cellular automata (CA) described by Nagel and Schreckenberg (NaSch). 
The NaSch CA model splits agent (vehicle) actions into four stages:
1. Acceleration
2. Braking
3. Randomisation
4. Vehicle Movement

In this implementation the 4th action is separated from the other actions to simulate simultaneous activation of the agents.
This isn't strictly necessary for non-multithreaded processes but ensures that vehicle positions wouldn't cause conflicts if it were multithreaded.

The model is written in Python using the Mesa ABM framework which allows for easy visualisation.

The vehicle density and the max speed are adjustable parameters in the visualisation. 
By default, the vehicle density is 0.2, and the max speed is 4.

This implementation was developed by studying Jackie Kazil's Mesa implementation of the Schelling Segregation Model.

## Installation

Mesa requires Python 3.6+. 
It's recommended that you use a virtual-environment, so you don't modify your base installation of Python. 

To install the additional dependencies use pip and the requirements.txt in this directory. 


```
    $ pip install -r requirements.txt
```

## How to Run

To run the model interactively, run the following in this directory:

```
    $ mesa runserver
```
 
Then open your browser to [http://127.0.0.1:8555/](http://127.0.0.1:8555/) (some interpreters/IDEs will do this for you). 
To run the model press Reset, adjust the frame rate, vehicle density and max speed as required, then press Start.

## Files

* [run.py](run.py): Launches the model visualization server. Can be run using the command ``mesa runserver``.
* [model.py](model.py): Contains the agent class, and the overall model class.
* [server.py](server.py): Defines classes for visualizing the model in the browser via Mesa's modular server, and instantiates a visualization server.
* [analysis.ipynb](analysis.ipynb): Jupyter notebook demonstrating basic model usage including a parameter sweep.

## Further Reading

Nagel and Schreckenberg's original paper describing the model:

[K. Nagel and M. Schreckenberg, ‘A cellular automaton model for freeway traffic,’ Journal de physique I, vol. 2, no. 12, pp. 2221–2229, 1992.](https://pdfs.semanticscholar.org/6e47/833b8d566f4b2edff695939241da2289ccc9.pdf?)

