# StellarAI 🧠🏁

StellarAI powers the autonomous navigation of the mobile robot *Hyperion*. This project was developed during the interdisciplinary course *Produktentwicklung (PREN)* at the Hochschule Luzern. Due to the COVID-19 pandemic, the functionality could only be simulated. However, the concepts used can be used for real-world robotics.

<p align="center">
  <img width="500px" src="https://github.com/strfx/stellar/blob/main/docs/simulator.png?raw=true" alt="Stellar viz"/>
</p>

StellarAI is inspired by the concepts taught in the Udacity course [Artificial Intelligence for Robotics](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373) and the book [Probabilistic Robotics](http://probabilistic-robotics.org/), including 
* [Occupancy Grid Maps](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) to transform the robot's physical environment into a probabilistic model
* [A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) to plan an optimal path through the environment (from start to finish)
* [Gradient descent](https://en.wikipedia.org/wiki/Gradient_descent) to optimize the planned path for the robots motion
* [PID controller](https://en.wikipedia.org/wiki/PID_controller) to stay on the planned track during motion
* [Computer vision](https://en.wikipedia.org/wiki/Computer_vision) to detect pylons in its environment

# Getting started

## Installation

To install Stellar, run (preferably in a virtualenv):

```sh
$ python setup.py install
```

## Dependencies
- Python 3
- Pipenv

Install all (development-) dependencies
```sh
$ pipenv install --dev
```

## Run tests

This project is using the pytest framework. To run the tests:
```sh
$ pipenv run python -m pytest tests/
```

# Components

StellarAI makes use of many components to gain autonomy. These components can be found in individual modules in the `stellar` package.

## Perception

The robot needs information about its environment. Perception collects these data from sensors and turns them into concrete signals.

Aims to solve following problems:
* Receive information about the robots environment
* Order sensor signals in chronologically
* Abstract the underlying hardware

## Cognition

Consists of the main logic; behaviour to achieve the overall-goals. 

Aims to solve following problems:
* Mapping of the environment
* Localizing the robot in the environment
* Plan an optimal path through the parcours
* Track robot on the planned path

## Action

Translates calculated next state into concrete signals for the underlying hardware.

Aims to solve following problems:
* Send signals to the engine as well as steering
* Abstracts underlying hardware


## Observatory

Gathers debug- and diagnostic data in order to introspect Stellar's behavior.

Aims to solve following problems:
* Allowing introspection into the inner-workings of stellar
* Forward information to the debug UI.

Run the Observatory server:

```sh
$ pipenv run python stellar/observatory/server.py
```

## Communication

Provides communication infrastructure between modules to allow e.g., gathering diagnostic information from different components.

# Contributors

* [strfx](https://github.com/strfx)
* [nolonar](https://github.com/nolonar)

# Credits

This project was inspired and uses code and techniques from many other projects, which are credited here:

* [PyRoboViz](https://github.com/simondlevy/PyRoboViz): Used to visualize the robot and the track.
* [occupancy-grid-a-star](https://github.com/richardos/occupancy-grid-a-star): Helped us greatly with the implementation of the occupancy grid mapping and the A* planning algorithm.

Thanks to all of you!
