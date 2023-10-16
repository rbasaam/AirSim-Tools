from utils import *
import numpy as np 
import threading

# Pull Frames Flag
SAVE_IMGS = True
NUM_FRAMES = 200
FRAME_RATE = 10 # fps

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
        surveyAltitude=30,
        sweepAmplitude=50,
        numSweeps=4,
        numWaypoints=120,
        plotFlag=True,
    )

    # Generate Random Spawn Points for Child Drones
    droneSpawnPoints = droneSpawn(
        waypoints=droneWaypoints,
        numDrones=2,
        FOV=np.array([10, 90, 60]),
        plotFlag=True,
    )

    # Create a thread for flying the waypoints
    playerSpeed = 20.0
    fly_thread = threading.Thread(target=flyWaypoints, args=(droneWaypoints, playerSpeed))
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
