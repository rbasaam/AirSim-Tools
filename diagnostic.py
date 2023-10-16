from utils import *
import numpy as np

# Define the Points of Interest in Unreal coordinates
baseballDiamond = np.array([[-18045, 24560, 320]]) # Baseball Diamond
lakeFountain = np.array([[3000, 5000, -430]]) # Lake
tennisCourt = np.array([[-9400, -38390, 90]]) # Tennis Courts
farField = np.array([[-84820, -15650, 10]]) # Far Field

# Concatenate the POIs into a single array
worldPOIs = np.concatenate(
    (
        baseballDiamond, 
        lakeFountain, 
        tennisCourt, 
        farField, 
    )
)

# Define the Labels for the POIs
poiLabels = [
    "Baseball Diamond",
    "Lake Fountain",
    "Tennis Court",
    "Far Field",
]

# Create the Waypoints for the Parent Drone to fly
droneWaypoints = POIPath(
    POIs=worldPOIs,
    POI_Labels=poiLabels,
    surveyAltitude=30,
    ySweep=0,
    sideSweeps=0,
    numWaypoints=200,
    plotFlag=True,
)

# Generate Random Spawn Points for Child Drones
droneSpawnPoints = droneSpawn(
    waypoints=droneWaypoints,
    numDrones=2,
    FOV=np.array([10, 90, 60]),
    plotFlag=True,
)

