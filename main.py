from utils import *
import numpy as np 
import threading


SAVE_IMGS = True

def main():
    # Define the start and end positions in Unreal coordinates
    unrealStart = np.array([[-18045, 24560, 320]]) # Baseball Diamond
    # unrealEnd = np.array([[3000, 5000, -430]]) # Lake
    # unrealEnd = np.array([[-9400, -38390, 90]]) # Tennis Courts
    unrealEnd = np.array([[-84820, -15650, 10]]) # Far Field
    # Define the FOV
    fov = np.array([10, 90, 60])

    # generate the path
    wayPoints = surveryFlightPath(
        playerStart = unrealStart, 
        playerEnd = unrealEnd, 
        surveryAltitude = 40.0, 
        ySweep=20,
        sideSweeps = 3,
        zSweep = 0, 
        altSweeps = 0, 
        numWaypoints = 1000,
        plotFlag = False,
        )
    
    # calculate spawn points for children drone actors
    spawnPoints = droneSpawn(
        waypoints=wayPoints,
        numDrones=5,
        FOV=fov,
        plotFlag=False,
    ) 

    # Create a thread for flying the waypoints
    playerSpeed = 20.0
    fly_thread = threading.Thread(target=flyWaypoints, args=(wayPoints, playerSpeed))
    fly_thread.start()

    if SAVE_IMGS:
        # Capture Data to Save Folders
        pullFrames(
            numFrames=25,
            timeInterval=3,
            saveFolder="saved_imgs/"
        )  
    # Wait for the fly_thread to finish
    fly_thread.join()    

if __name__ == "__main__":
    main()
