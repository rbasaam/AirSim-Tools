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

#### `POIPath(POIs, POI_Labels, surveyAltitude, ySweep, sideSweeps, numWaypoints, plotFlag=False)`

Generates a path that visits a set of Points of Interest (POIs).

- `POIs`: Array of POI coordinates (numpy.ndarray).
- `POI_Labels`: List of POI labels (list).
- `surveyAltitude`: Altitude at which to survey the POIs (np.float32).
- `ySweep`: Amplitude of the side-to-side sweep (np.float32).
- `sideSweeps`: Number of side-to-side sweeps (np.float32).
- `numWaypoints`: Number of waypoints per segment (np.uint8).
- `plotFlag` (optional): Flag indicating whether to plot the path (bool, default=False).

Returns

- `droneWaypoints`: Array of waypoints representing the survey path (np.ndarray).

#### `flyWaypoints(waypoints, playerSpeed)`

Flies the multirotor along a given set of waypoints.

- `waypoints`: Array of waypoints representing the flight path (numpy.ndarray).
- `playerSpeed`: Speed at which the multirotor should fly (np.float32).

#### `pullFrames(numFrames, timeInterval, saveFolder)`

Pulls a specified number of frames from the AirSim simulator and saves them to the specified folder.

- `numFrames`: Number of frames to pull (np.uint8).
- `timeInterval`: Time interval between each frame pull (np.float32).
- `saveFolder`: Path to the folder where the frames will be saved (str).

#### `droneSpawn(waypoints, numDrones, FOV, plotFlag=False)`

Generates spawn points for a specified number of drones at each waypoint within the chief drone's Field of View (FOV).

- `waypoints`: Array of waypoints representing the flight path (numpy.ndarray).
- `numDrones`: Number of drones to spawn at each waypoint (np.uint8).
- `FOV`: Field of view of the chief drone in the format [range, theta, phi] (np.array).
- `plotFlag` (optional): Flag indicating whether to plot the spawn points (bool, default=False).

Returns

- `spawnPoints`: Array of spawn points for each drone at each waypoint (np.ndarray).

#### `getPathTangents(waypoints)`

Calculates the tangent vectors for each waypoint in a given path.

- `waypoints`: Array of waypoints representing the flight path (numpy.ndarray).

Returns

- `tangentVectors`: Array of tangent vectors for each waypoint (np.ndarray).

#### `spherical2cartesian(r, theta, phi)`

Converts spherical coordinates to Cartesian coordinates.

- `r`: Radius (np.float32).
- `theta`: Theta angle (np.float32).
- `phi`: Phi angle (np.float32).

Returns

- `Cartesian coordinates`: Array of Cartesian coordinates (np.ndarray).

#### `plotFOVs(waypoints, spawnpoints, FOV, waypointIndex, fig=None)`

Plots the Field of View (FOV) for a specified waypoint and the spawn points for the drones at that waypoint.

- `waypoints`: Array of waypoints representing the flight path (numpy.ndarray).
- `spawnpoints`: Array of spawn points for each drone at each waypoint (np.ndarray).
- `FOV`: Field of view of the chief drone in the format [range, theta, phi] (np.array).
- `waypointIndex`: Index of the waypoint for which to plot the FOV and spawn points (np.uint8).
- `fig` (optional): Matplotlib figure object to which to add the plot (matplotlib.figure.Figure, default=None).
### `main.py`

The `main.py` script demonstrates the usage of the functions in `utils.py`. To run the script, execute the following command:

```bash
python main.py
```

Make sure to modify the parameters in the `main()` function to suit your needs.
