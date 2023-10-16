from utils import *
import numpy as np 
import threading

# Pull Frames Flag
SAVE_IMGS = False # Save Images to Disk
NUM_FRAMES = 200 # Number of Frames to Save
FRAME_RATE = 10 # fps

# Flight Path Parameters
PLAYER_SPD = 20 # m/s
SURVEY_ALT = 30 # m
SWEEP_AMP = 50 # m
NUM_SWEEPS = 1 # Number of Sweeps
NUM_WAYPOINTS = 120 # Number of Waypoints

# Drone Spawn Parameters
NUM_DRONES = 2 # Number of Drones to Spawn at each waypoint
FOV = np.array([10, 90, 60]) # Field of View of the Chief Drone

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

# Define the main function
def main():

    # Create the Waypoints for the Chief Drone to fly
    droneWaypoints = POIPath(
        POIs=worldPOIs,
        POI_Labels=poiLabels,
        surveyAltitude=SURVEY_ALT,
        sweepAmplitude=SWEEP_AMP,
        numSweeps=NUM_SWEEPS,
        numWaypoints=NUM_WAYPOINTS,
        plotFlag=True,
    )

    # Generate Random Spawn Points for Child Drones
    droneSpawnPoints = droneSpawn(
        waypoints=droneWaypoints,
        numDrones=NUM_DRONES,
        FOV=FOV,
    )

    fly_thread = threading.Thread(target=flyWaypoints, args=(droneWaypoints, PLAYER_SPD))
    fly_thread.start()

    if SAVE_IMGS:
        # Capture Data to Save Folders
        pullFrames(
            numFrames=NUM_FRAMES,
            frameRate=FRAME_RATE,
            saveFolder="AirSim-Tools/saved_imgs/"
        )  
    # Wait for the fly_thread to finish
    fly_thread.join()    

if __name__ == "__main__":
    main()
