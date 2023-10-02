import airsim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import os


def surveryFlightPath(playerStart, playerEnd, surveryAltitude, ySweep, sideSweeps, zSweep, altSweeps, numWaypoints, plotFlag=False):
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

    playerTrip = playerEnd - playerStart

    # generate the path
    if numWaypoints>1000:
        xx = np.linspace(0, playerTrip[0], numWaypoints)
        yy = np.linspace(0, playerTrip[1], numWaypoints)+ySweep*np.sin((xx/playerTrip[0])*2*np.pi*sideSweeps)
        zz = np.linspace(0, playerTrip[2], numWaypoints)-surveryAltitude-zSweep*np.sin((xx/playerTrip[0])*2*np.pi*altSweeps)
    else:
        xx = np.linspace(0, playerTrip[0], 1000)
        yy = np.linspace(0, playerTrip[1], 1000)+ySweep*np.sin((xx/playerTrip[0])*2*np.pi*sideSweeps)
        zz = np.linspace(0, playerTrip[2], 1000)-surveryAltitude-zSweep*np.sin((xx/playerTrip[0])*2*np.pi*altSweeps)

    # convert to waypoints
    waypoints = np.array([xx, yy, zz]).T - playerStart

    # select waypoints from the path
    indices = np.linspace(0, waypoints.shape[0]-1, numWaypoints).astype(int)
    waypoints = waypoints[indices]

    # plot the path
    if plotFlag:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.text(waypoints[0,0], waypoints[0,1], waypoints[0,2], 'start')
        ax.text(waypoints[-1,0], waypoints[-1,1], waypoints[-1,2], 'end')
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
    client.enableApiControl(True)
    client.armDisarm(True)
    print("connected!")

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

    # land
    client.landAsync().join()
    print("landed!")

     # cleanup
    client.armDisarm(False)
    client.enableApiControl(False)
    print("disconnected!")

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