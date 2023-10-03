import airsim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import os


def surveryFlightPath(
        playerStart: np.ndarray, 
        playerEnd: np.ndarray, 
        surveryAltitude: np.float32, 
        ySweep: np.float32, 
        sideSweeps: np.float32, 
        zSweep: np.float32, 
        altSweeps: np.float32, 
        numWaypoints: np.uint8, 
        plotFlag=False
        ):
    """
    Generates a flight path for surveying an area between two ponp.uint8s.

    Args:
        playerStart (numpy.ndarray): Starting position of the player.
        playerEnd (numpy.ndarray): Ending position of the player.
        surveryAltitude (np.float32): Altitude at which the survey is conducted.
        ySweep (np.float32): Amplitude of the sinusoidal sweep in the y-direction.
        sideSweeps (np.float32): Number of side sweeps to perform.
        zSweep (np.float32): Amplitude of the sinusoidal sweep in the z-direction.
        altSweeps (np.float32): Number of altitude sweeps to perform.
        numWaypoints (np.uint8): Number of waypoints to generate.
        plotFlag (bool, optional): Flag indicating whether to plot the flight path. Defaults to False.

    Returns:
        np.ndarray: Array of waypoints representing the flight path.
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
        
    return waypoints

def flyWaypoints(waypoints: np.ndarray, playerSpeed: np.float32):
    """
    Flies the multirotor along a given set of waypoints.

    Args:
        waypoints (numpy.ndarray): Array of waypoints representing the flight path.
        playerSpeed (np.float32): Speed at which the multirotor should fly.

    Returns:
        None
    """
    # convert to airsim.Vector3r
    waypoints = [airsim.Vector3r(x[0], x[1], x[2]) for x in waypoints]

    # connect to airsim
    print("connecting...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("connected!")
    client.enableApiControl(True)
    client.armDisarm(True)
    print("arming...")
    print(f"FOV: {client.simGetCurrentFieldOfView(camera_name='0')}")
    print(f"Vehicle Name: {client.listVehicles()[0]}")

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
        adaptive_lookahead = 1,
        vehicle_name = 'SimpleFlight'
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
        adaptive_lookahead = 1,
        vehicle_name = 'SimpleFlight'
        ).join()
    # land
    print("landing...")
    client.landAsync(
        timeout_sec=10,
        vehicle_name = 'SimpleFlight'
    ).join()
    # Confirm landed before disconnecting
    time.sleep(10)
    print("landed!")
    time.sleep(5)

    """     
    # cleanup
    print("disarming...")
    client.armDisarm(False)
    print("disarmed!")

    print("disconnecting...")
    client.enableApiControl(False)
    print("disconnected!")   
    """

    return

def pullFrames(numFrames: np.uint8, timeInterval: np.float32, saveFolder: str):
    """
    Pulls a specified number of frames from the AirSim simulator and saves them to the specified folder.

    Args:
        numFrames (int): Number of frames to pull.
        timeInterval (np.float32): Time interval between each frame pull.
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

def droneSpawn(waypoints: np.ndarray, numDrones: np.uint8, FOV: np.array, plotFlag=False):
    """
    Generates spawn points for a specified number of drones at each waypoint within the chief drones FOV.

    Args:
        waypoints (numpy.ndarray): Array of waypoints representing the flight path.
        numDrones (np.uint8): Number of drones to spawn at each waypoint.
        FOV (np.array): Field of view of the chief drone in the format [range, theta, phi].
        plotFlag (bool, optional): Flag indicating whether to plot the spawn points. Defaults to False.

    Returns:
        np.ndarray: Array of spawn points for each drone at each waypoint.
    """
    # Calculate Tangent Vectors for each waypoint
    tangentVectors = getPathTangents(waypoints)
    # Initialize Spawn Points Array
    spawnPoints = np.zeros((numDrones, 3, waypoints.shape[0]))

    # Generate random spawn points for each waypoint
    for wp in range(len(waypoints)):
        # Generate random ranges
        ranges = FOV[0]*np.random.rand(numDrones)
        # Generate random Theta Angles centered around the tangent vector
        tangentTheta = np.arctan2(tangentVectors[wp,1], tangentVectors[wp,0])
        thetaRange = FOV[1]/2
        thetas = tangentTheta + np.deg2rad(-thetaRange + (np.random.rand(numDrones)*thetaRange*2))
        # Generate random Phi Angles centered around the tangent vector
        tangentPhi = np.arctan2(tangentVectors[wp, 2], np.sqrt(tangentVectors[wp, 0]**2 + tangentVectors[wp, 1]**2))
        phiRange = FOV[2] / 2  
        phis = np.pi / 2 + tangentPhi + np.deg2rad(-phiRange + (2 * phiRange * np.random.rand(numDrones)))
        # Convert spherical coordinates to Cartesian and add them to waypoints
        for drone in range(numDrones):
            spawnPoints[drone,:,wp] = waypoints[wp] + spherical2cartesian(ranges[drone], thetas[drone], phis[drone])

    if plotFlag:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.text(waypoints[0,0], waypoints[0,1], waypoints[0,2], 'pathStart')
        ax.text(waypoints[-1,0], waypoints[-1,1], waypoints[-1,2], 'pathEnd')        
        ax.azim = -80 
        ax.elev = 18 
        ax.set_title('Flight Path World Frame')

        for wp in range(len(waypoints)):
            for drone in range(numDrones):
                ax.scatter(spawnPoints[drone,0,wp], spawnPoints[drone,1,wp], spawnPoints[drone,2,wp], color='r', marker='o')
        plt.show()

    return spawnPoints

def getPathTangents(waypoints: np.ndarray):
    """
    Calculates the tangent vectors for each waypoint in a given path.

    Args:
        waypoints (numpy.ndarray): Array of waypoints representing the flight path.

    Returns:
        np.ndarray: Array of tangent vectors for each waypoint.
    """
    # Calculate Tangent Vectors for each waypoint
    tangentVectors = np.diff(waypoints, axis=0)
    tangentVectors = np.concatenate((tangentVectors, tangentVectors[-1:]), axis=0)

    # Normalize Tangent Vectors
    tangentVectors = tangentVectors/np.linalg.norm(tangentVectors, axis=1)[:,None]
    return tangentVectors

def spherical2cartesian(r: np.float32, theta: np.float32, phi: np.float32):
    """
    Converts spherical coordinates to Cartesian coordinates.

    Args:
        r (np.float32): Radius.
        theta (np.float32): Theta angle.
        phi (np.float32): Phi angle.

    Returns:
        np.ndarray: Array of Cartesian coordinates.
    """
    x = r * np.cos(theta) * np.sin(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(phi)
    return np.array([x, y, z])