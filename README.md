# AirSim Tools

This repository contains a set of tools for working with the AirSim simulator. The tools are implemented in Python and utilize the AirSim API.

## Installation

Clone the repository:

```bash
git clone https://github.com/rbasaam/AirSim-Tools.git
```

Install the required dependencies:

- [AirSim](https://github.com/AVL-TMU/UnrealEngine-AirSim-PX4.git)
- numpy
- matplotlib

## Usage

### `utils.py`

The `utils.py` module provides several utility functions for working with the AirSim simulator. Here are the available functions:

#### `surveryFlightPath(playerStart, playerEnd, surveryAltitude, ySweep, sideSweeps, zSweep, altSweeps, numWaypoints, plotFlag=False)`

Generates a flight path for surveying an area between two points.

- `playerStart`: Starting position of the player (numpy.ndarray).
- `playerEnd`: Ending position of the player (numpy.ndarray).
- `surveryAltitude`: Altitude at which the survey is conducted (float).
- `ySweep`: Amplitude of the sinusoidal sweep in the y-direction (float).
- `sideSweeps`: Number of side sweeps to perform (float).
- `zSweep`: Amplitude of the sinusoidal sweep in the z-direction (float).
- `altSweeps`: Number of altitude sweeps to perform (float).
- `numWaypoints`: Number of waypoints to generate (int).
- `plotFlag` (optional): Flag indicating whether to plot the flight path (bool, default=False).

Returns

- `waypoints`: List of airsim.Vector3r waypoints representing the flight path.

#### `flyWaypoints(waypoints, playerSpeed)`

Flies the multirotor along a given set of waypoints.

- `waypoints`: Array of waypoints representing the flight path (numpy.ndarray).
- `playerSpeed`: Speed at which the multirotor should fly (float).

#### `pullFrames(numFrames, timeInterval, saveFolder)`

Pulls a specified number of frames from the AirSim simulator and saves them to the specified folder.

- `numFrames`: Number of frames to pull (int).
- `timeInterval`: Time interval between each frame pull (float).
- `saveFolder`: Path to the folder where the frames will be saved (str).

### `main.py`

The `main.py` script demonstrates the usage of the functions in `utils.py`. To run the script, execute the following command:

```bash
python main.py
```

Make sure to modify the parameters in the `main()` function to suit your needs.
