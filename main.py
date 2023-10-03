from utils import *
import numpy as np 
import threading


SAVE_IMGS = False

def main():
    unrealStart = np.array([[-18045, 24560, 320]])
    unrealEnd = np.array([[3000, 5000, -430]])
    # generate the path
    waypoints = surveryFlightPath(
        playerStart = unrealStart, 
        playerEnd = unrealEnd, 
        surveryAltitude = 40.0, 
        ySweep=50,
        sideSweeps = 10,
        zSweep = 0, 
        altSweeps = 0, 
        numWaypoints = 1200,
        plotFlag = True,
        )
    
    # Create a thread for flying the waypoints
    playerSpeed = 20.0
    fly_thread = threading.Thread(target=flyWaypoints, args=(waypoints, playerSpeed))
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
