from utils import *
import numpy as np 
import threading


SAVE_IMGS = False

def main():
    # Define the start and end positions in Unreal coordinates
    baseballDiamond = np.array([[-18045, 24560, 320]]) # Baseball Diamond
    lakeFountain = np.array([[3000, 5000, -430]]) # Lake
    tennisCourt = np.array([[-9400, -38390, 90]]) # Tennis Courts
    farField = np.array([[-84820, -15650, 10]]) # Far Field

    worldPOIs = np.concatenate(
        (
            baseballDiamond, 
            lakeFountain, 
            tennisCourt, 
            farField, 
        )
    )

    poiLabels = [
        "Baseball Diamond",
        "Lake Fountain",
        "Tennis Court",
        "Far Field"
    ]


    droneWaypoints = POIPath(
        POIs=worldPOIs,
        POI_Labels=poiLabels,
        surveyAltitude=30,
        ySweep=20,
        sideSweeps=3,
        numWaypoints=500,
        plotFlag=True
    )
    
    """
    # Define the FOV
    fov = np.array([10, 90, 60])

    # calculate spawn points for children drone actors
    spawnPoints = droneSpawn(
        waypoints=droneWaypoints,
        numDrones=5,
        FOV=fov,
        plotFlag=False,
    ) 
    """

    # Create a thread for flying the waypoints
    playerSpeed = 20.0
    fly_thread = threading.Thread(target=flyWaypoints, args=(droneWaypoints, playerSpeed))
    fly_thread.start()

    if SAVE_IMGS:
        # Capture Data to Save Folders
        pullFrames(
            numFrames=50,
            timeInterval=3,
            saveFolder="saved_imgs/"
        )  
    # Wait for the fly_thread to finish
    fly_thread.join()    

if __name__ == "__main__":
    main()
