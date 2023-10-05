import airsim
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import pprint

def POIPath(
        POIs: np.ndarray,
        POI_Labels: list,
        surveyAltitude: np.float32,
        ySweep: np.float32,
        sideSweeps: np.float32,
        numWaypoints: np.uint8,
        plotFlag=False
        ):

    """
    Generate a path that visits a set of POIs.

    """ 
    dronePOIs = (POIs - POIs[0,:])*np.array([0.01, 0.01, -0.01])
    segmentDirections = np.diff(dronePOIs, axis=0)
    numSegments = segmentDirections.shape[0]
    dronePath = np.zeros((numWaypoints, 3, numSegments))
    for segment in range(numSegments):
        if numWaypoints > 1000:
            xx = np.linspace(dronePOIs[segment,0], dronePOIs[segment+1,0], numWaypoints)
            yy = np.linspace(dronePOIs[segment,1], dronePOIs[segment+1,1], numWaypoints)+ySweep*np.sin((xx/segmentDirections[segment,0])*2*np.pi*sideSweeps)
            zz = np.linspace(dronePOIs[segment,2], dronePOIs[segment+1,2], numWaypoints)-surveyAltitude
        else:
            xx = np.linspace(dronePOIs[segment,0], dronePOIs[segment+1,0], 1000)
            yy = np.linspace(dronePOIs[segment,1], dronePOIs[segment+1,1], 1000)+ySweep*np.sin((xx/segmentDirections[segment,0])*2*np.pi*sideSweeps)
            zz = np.linspace(dronePOIs[segment,2], dronePOIs[segment+1,2], 1000)-surveyAltitude
        # select waypoints from the path
        waypoints = np.array([xx, yy, zz]).T
        indices = np.linspace(0, waypoints.shape[0]-1, numWaypoints).astype(int)
        waypoints = waypoints[indices]
        dronePath[:,:,segment] = waypoints
    droneWaypoints = dronePath.reshape(numWaypoints*numSegments, 3)

    if plotFlag:
        fig = plt.figure()

    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(POIs[:,0], POIs[:,1], POIs[:,2], c='r', marker='o')
    for i in range(len(segmentDirections)):
        ax.quiver(POIs[i,0], POIs[i,1], POIs[i,2], segmentDirections[i,0]*100, segmentDirections[i,1]*100, segmentDirections[i,2]*-100, color='b', length=0.99, arrow_length_ratio=0.01)
    for i, text in enumerate(POI_Labels):
        ax.text(POIs[i,0], POIs[i,1], POIs[i,2], text)
    ax.azim = 150
    ax.elev = -150
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(dronePOIs[:,0], dronePOIs[:,1], dronePOIs[:,2], c='r', marker='o')
    for i, text in enumerate(POI_Labels):
        ax2.text(dronePOIs[i,0], dronePOIs[i,1], dronePOIs[i,2], text)
    for segment in range(numSegments):
        ax2.plot(dronePath[:,0,segment], dronePath[:,1,segment], dronePath[:,2,segment])
    ax2.azim = -120
    ax2.elev = -120
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    plt.show()

    return droneWaypoints

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
    Generates a flight path for surveying an area between two points.

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
        # Plot the flight path in the world frame
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

        # Plot the flight path in the drone frame
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

    state = client.getMultirotorState(vehicle_name='SimpleFlight')
    s = pprint.pformat(state)
    print("state: %s" % s)

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

    time.sleep(5)  
     
    client.moveByVelocityZAsync(
        vx=0,
        vy=0,
        z=waypoints[-1].z_val+40,
        duration=10,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(False,0),
        vehicle_name = 'SimpleFlight'
        ).join()
    
    time.sleep(5)
    
    # land
    print("landing...")
    client.landAsync(
        timeout_sec=10,
        vehicle_name = 'SimpleFlight'
    ).join()
    # Confirm landed before disconnecting
    print("landed!")
    time.sleep(5)
     
    # cleanup
    print("disarming...")
    client.armDisarm(False)
    print("disarmed!")

    print("disconnecting...")
    client.enableApiControl(False)
    print("disconnected!")   

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
        airsim.write_file(os.path.join(povFolder, f"pov_{imageIndex+i}.png"), responses[0].image_data_uint8)
        airsim.write_file(os.path.join(depthFolder, f"depth_{imageIndex+i}.png"), responses[1].image_data_uint8)
        airsim.write_file(os.path.join(maskFolder, f"mask_{imageIndex+i}.png"), responses[2].image_data_uint8)

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

    radius = FOV[0]
    thetaRange = np.deg2rad(FOV[1])
    phiRange = np.deg2rad(FOV[2])

    # Generate random spawn points for each waypoint
    for wp in range(len(waypoints)):
        # Generate random ranges
        ranges = np.random.uniform(3, FOV[0], size=numDrones)
        # Generate random Theta Angles centered around the tangent vector
        tangentTheta = np.arctan2(tangentVectors[wp,1], tangentVectors[wp,0])
        thetas = tangentTheta + (-thetaRange + (np.random.rand(numDrones)*thetaRange*2))
        # Generate random Phi Angles centered around the tangent vector
        tangentPhi = np.arctan2(tangentVectors[wp, 2], np.sqrt(tangentVectors[wp, 0]**2 + tangentVectors[wp, 1]**2)) 
        phis = np.pi / 2 + tangentPhi + (-phiRange + (2 * phiRange * np.random.rand(numDrones)))
        # Convert spherical coordinates to Cartesian and add them to waypoints
        for drone in range(numDrones):
            spawnPoints[drone,:,wp] = waypoints[wp] + spherical2cartesian(ranges[drone], thetas[drone], phis[drone])

    if plotFlag:
        waypointIndex=np.random.randint(low=0, high=len(waypoints) - 1)

        fig = plt.figure()

        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.set_box_aspect([1,1,1])
        ax.plot(waypoints[:,0], waypoints[:,1], waypoints[:,2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.text(waypoints[0,0], waypoints[0,1], waypoints[0,2], 'pathStart')
        ax.text(waypoints[-1,0], waypoints[-1,1], waypoints[-1,2], 'pathEnd')        
        ax.azim = -80 
        ax.elev = 18 
        ax.set_title('Random Spawn Points Along Flight Path in World Frame')
        
        for wp in range(len(waypoints)):
            for drone in range(numDrones):
                ax.scatter(spawnPoints[drone,0,wp], spawnPoints[drone,1,wp], spawnPoints[drone,2,wp], color='r', marker='o')

        plotFOVs(
            waypoints=waypoints,
            spawnpoints=spawnPoints,
            FOV=FOV,
            waypointIndex=waypointIndex,
            fig=fig,
        )

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

def spherical2cartesian(r, theta, phi):
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

def plotFOVs(waypoints: np.ndarray, spawnpoints: np.ndarray, FOV: np.array, waypointIndex: np.uint8, fig=None):
    # Calculate Tangent Vectors all Waypoints
    tangentVectors = getPathTangents(waypoints)
    radius = FOV[0]
    thetaRange = np.deg2rad(FOV[1])
    phiRange = np.deg2rad(FOV[2])
    # Calculate Normal Vectors for each waypoint
    center = waypoints[waypointIndex]
    tangent = tangentVectors[waypointIndex]
    # Calculate Waypoint Heading
    tangentTheta = np.arctan2(tangent[1], tangent[0])
    tangentPhi = np.arctan2(tangent[2], np.sqrt(tangent[0]**2 + tangent[1]**2))+np.pi/2
    # Calculate FOV
    spherePoints = 100
    theta  = np.linspace(tangentTheta-thetaRange/2, tangentTheta+thetaRange/2, spherePoints)
    phi   = np.linspace(tangentPhi-phiRange/2, tangentPhi+phiRange/2, spherePoints)
    theta, phi = np.meshgrid(theta, phi)
    # Calculate FOV Points
    x = center[0] + radius*np.sin(phi)*np.cos(theta)
    y = center[1] + radius*np.sin(phi)*np.sin(theta)
    z = center[2] + radius*np.cos(phi)
    if fig is not None:
        # Plot
        ax = fig.add_subplot(1,2,2,projection='3d')
        ax.plot(center[0], center[1], center[2], 'bo')
        ax.quiver(center[0], center[1], center[2], tangent[0], tangent[1], tangent[2], length=radius, normalize=True, color='r')
        ax.scatter(spawnpoints[:,0,waypointIndex], spawnpoints[:,1,waypointIndex], spawnpoints[:,2,waypointIndex], 'go')
        ax.plot_surface(x, y, z, cmap='viridis', alpha=0.5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect([1,1,1])
        ax.set_title(f"Waypoint {waypointIndex} FOV and Random Spawn Points")
        ax.azim = -50
        ax.elev = 20
        plt.show()
    return

