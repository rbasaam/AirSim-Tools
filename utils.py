import airsim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import os


def surveryFlightPath(
        playerStart: np.ndarray, 
        playerEnd: np.ndarray, 
        surveryAltitude: float, 
        ySweep: float, 
        sideSweeps: float, 
        zSweep: float, 
        altSweeps: float, 
        numWaypoints: int, 
        plotFlag=False
        ):
    """
    Generates a flight path for surveying an area between two points.

    Args:
        playerStart (numpy.ndarray): Starting position of the player.
        playerEnd (numpy.ndarray): Ending position of the player.
        surveryAltitude (float): Altitude at which the survey is conducted.
        ySweep (float): Amplitude of the sinusoidal sweep in the y-direction.
        sideSweeps (float): Number of side sweeps to perform.
        zSweep (float): Amplitude of the sinusoidal sweep in the z-direction.
        altSweeps (float): Number of altitude sweeps to perform.
        numWaypoints (int): Number of waypoints to generate.
        plotFlag (bool, optional): Flag indicating whether to plot the flight path. Defaults to False.

    Returns:
        airsim.vector3r: Array of waypoints representing the flight path.
    """
    # Convert to meters
    playerStart = playerStart/100
    playerEnd = playerEnd/100
    playerTrip = playerEnd - playerStart

    # Convert to Drone Frame
    playerTrip = playerTrip*np.array([1, 1, -1])

    # generate the path wrt Drone Frame
    if numWaypoints>1000:
        xx = np.linspace(0, playerTrip[0,0], numWaypoints)
        yy = np.linspace(0, playerTrip[0,1], numWaypoints)+ySweep*np.sin((xx/playerTrip[0,0])*2*np.pi*sideSweeps)
        zz = np.linspace(0, playerTrip[0,2], numWaypoints)-surveryAltitude-zSweep*np.sin((xx/playerTrip[0,0])*2*np.pi*altSweeps)
    else:
        xx = np.linspace(0, playerTrip[0,0], 1000)
        yy = np.linspace(0, playerTrip[0,1], 1000)+ySweep*np.sin((xx/playerTrip[0,0])*2*np.pi*sideSweeps)
        zz = np.linspace(0, playerTrip[0,2], 1000)-surveryAltitude-zSweep*np.sin((xx/playerTrip[0,0])*2*np.pi*altSweeps)
    waypoints = np.array([xx, yy, zz]).T 
    
    # convert to path wrt to World Frame
    globalPath = (waypoints*np.array([1, 1, -1]) + playerStart)*100

    # select waypoints from the path
    indices = np.linspace(0, waypoints.shape[0]-1, numWaypoints).astype(int)
    waypoints = waypoints[indices]

    # plot the path
    if plotFlag:
        globalFlight = np.concatenate((
            # start
            playerStart*100,
            # hover
            playerStart*100 + np.array([[0, 0, surveryAltitude]]),
            # path
            globalPath,
            # hover
            playerEnd*100 + np.array([[0, 0, surveryAltitude]]),
            # end
            playerEnd*100,
        ))

        localFlight = np.concatenate((
            # start
            np.array([[0, 0, 0]]),
            # hover
            np.array([[0, 0, -surveryAltitude]]),
            # path
            waypoints,
            # hover
            playerTrip + np.array([[0, 0, -surveryAltitude]]),
            # End
            playerTrip,
        ))

        fig = plt.figure()
        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot(globalFlight[:,0], globalFlight[:,1], globalFlight[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.text(globalFlight[0,0], globalFlight[0,1], globalFlight[0,2], 'playerStart')
        ax.text(globalFlight[1,0], globalFlight[1,1], globalFlight[1,2], 'pathStart')
        ax.text(globalFlight[-2,0], globalFlight[-2,1], globalFlight[-2,2], 'pathEnd')        
        ax.text(globalFlight[-1,0], globalFlight[-1,1], globalFlight[-1,2], 'playerEnd')
        ax.azim = -80 
        ax.elev = 18 
        ax.set_title('Flight Path World Frame')

        print(f"Player Start: {globalFlight[0,:]}")
        print(f"Path Start: {globalFlight[1,:]}")
        print(f"Path End: {globalFlight[-2,:]}")
        print(f"Player End: {globalFlight[-1,:]}")

        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.plot(localFlight[:,0], localFlight[:,1], localFlight[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.text(localFlight[0,0], localFlight[0,1], localFlight[0,2], 'playerStart')
        ax.text(localFlight[1,0], localFlight[1,1], localFlight[1,2], 'pathStart')
        ax.text(localFlight[-2,0], localFlight[-2,1], localFlight[-2,2], 'pathEnd')        
        ax.text(localFlight[-1,0], localFlight[-1,1], localFlight[-1,2], 'playerEnd') 
        ax.azim = -130 
        ax.elev = -160 
        ax.set_title('Flight Path Drone Frame')

        plt.show()

    # convert to airsim.Vector3r
    waypoints = [airsim.Vector3r(x[0], x[1], x[2]) for x in waypoints]
        
    return waypoints

def flyWaypoints(waypoints, playerSpeed):
    """
    Flies the multirotor along a given set of waypoints.

    Args:
        waypoints (numpy.ndarray): Array of waypoints representing the flight path.
        playerSpeed (float): Speed at which the multirotor should fly.

    Returns:
        None
    """

    # connect to airsim
    print("connecting...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("connected!")
    client.enableApiControl(True)
    client.armDisarm(True)
    print("arming...")
    print(f"Vehicle Name: {client.getMultirotorState().vehicle_name}")

    # takeoff
    client.takeoffAsync().join()
    print("took off!")

    # fly along the waypoints
    print("flying...")
    client.moveOnPathAsync(
        path = waypoints, 
        velocity = playerSpeed, 
        timeout_sec = 600, 
        drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom, 
        yaw_mode = airsim.YawMode(True,2), 
        lookahead = -1, 
        adaptive_lookahead = 1
        ).join()

    # move to the end of the path
    print("moving to end of path...")
    client.moveToPositionAsync(
        x=waypoints[-1].x_val, 
        y=waypoints[-1].y_val, 
        z=waypoints[-1].z_val, 
        velocity=5,
        timeout_sec=10,
        drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom, 
        yaw_mode = airsim.YawMode(True,2), 
        lookahead = -1, 
        adaptive_lookahead = 1
        ).join()
    # land
    print("landing...")
    client.landAsync().join()
    # Confirm landed before disconnecting
    time.sleep(10)
    print("landed!")


    """     
    # cleanup
    client.armDisarm(False)
    client.enableApiControl(False)
    print("disconnected!")
    """

    return

def pullFrames(numFrames: int, timeInterval: float, saveFolder: str):
    """
    Pulls a specified number of frames from the AirSim simulator and saves them to the specified folder.

    Args:
        numFrames (int): Number of frames to pull.
        timeInterval (float): Time interval between each frame pull.
        saveFolder (str): Path to the folder where the frames will be saved.

    Returns:
        None
    """

    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Ensure the folders exist
    povFolder = os.path.join(saveFolder, "pov")
    depthFolder = os.path.join(saveFolder, "depth")
    maskFolder = os.path.join(saveFolder, "mask")

    os.makedirs(saveFolder, exist_ok=True)
    os.makedirs(povFolder, exist_ok=True)
    os.makedirs(depthFolder, exist_ok=True)
    os.makedirs(maskFolder, exist_ok=True)

    imageIndex = len(os.listdir(povFolder))+1

    for i in range(numFrames):
        # Specify the image names
        pov_img_name   = f"pov_{imageIndex+i}.png"
        depth_img_name = f"depth_{imageIndex+i}.png"
        mask_img_name  = f"mask_{imageIndex+i}.png"

        # Get images from the POV, depth, and segmentation mask feeds
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene),  # POV
            airsim.ImageRequest("0", airsim.ImageType.DepthVis),  # Depth
            airsim.ImageRequest("0", airsim.ImageType.Segmentation)  # Segmentation mask
        ])

        # Save the images
        airsim.write_file(os.path.join(povFolder, f"pov_{i}.png"), responses[0].image_data_uint8)
        airsim.write_file(os.path.join(depthFolder, f"depth_{i}.png"), responses[1].image_data_uint8)
        airsim.write_file(os.path.join(maskFolder, f"mask_{i}.png"), responses[2].image_data_uint8)

        # Print the saved image names and folders
        print(f"Saved Image: {pov_img_name  } to {povFolder}")
        print(f"Saved Image: {depth_img_name} to {depthFolder}")
        print(f"Saved Image: {mask_img_name } to {maskFolder}")


        # Wait for the specified time interval before getting the next set of images
        time.sleep(timeInterval)

    return

def getPathTangents(path):
    tangents = np.zeros(path.shape)
    tangents[0] = path[1] - path[0]
    tangents[1:-1] = path[2:] - path[:-2]
    tangents[-1] = path[-1] - path[-2]
    return tangents

def spherical2cartesian(r, theta, phi):
    x = r * np.cos(theta) * np.sin(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(phi)
    return np.array([x, y, z])