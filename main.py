from utils import *
import numpy as np 
import threading

# Pull Frames Flag
SAVE_IMGS = True
NUM_FRAMES = 100
FRAME_RATE = 30

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
    "Far Field"
]

# Define the main function
def main():

    # Create the waypoints for the drone to fly
    droneWaypoints = POIPath(
        POIs=worldPOIs,
        POI_Labels=poiLabels,
        surveyAltitude=30,
        ySweep=20,
        sideSweeps=1,
        numWaypoints=200,
        plotFlag=False,
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
