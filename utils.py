import airsim
import numpy as np
import matplotlib.pyplot as plt
import time
import os
from icecream import ic

def unreal2dronePOIs(
        unrealPOIs: np.ndarray
    ):
    """
    Convert the Unreal POIs to Drone Coordinates
    """
    # Define the Transformation Matrix
    T = np.array([
        [0.01,  0.00,  0.00],
        [0.00,  0.01,  0.00],
        [0.00,  0.00, -0.01],
    ])

    # Convert the POIs
    dronePOIs = np.matmul(unrealPOIs, T)

    return dronePOIs

def p2pSweep(
        startPoint: np.ndarray, 
        endPoint: np.ndarray, 
        surveyAltitude: np.float32, 
        sweepAmplitude: np.float32, 
        numSweeps: np.uint8, 
        numWaypoints: np.uint8
    ):
    """
    Generate a path that sweeps from a start point to an end point.

    Args:
        startPoint (np.ndarray): Start point of the path.
        endPoint (np.ndarray): End point of the path.
        surveyAltitude (np.float32): Altitude at which to survey the POIs.
        sweepAmplitude (np.float32): Amplitude of the side-to-side sweep.
        numSweeps (np.float32): Number of side-to-side sweeps.
        numWaypoints (np.uint8): Number of waypoints per segment.
    
    Returns:
        np.ndarray: Array of waypoints representing the survey path.
    """
    # Calculate the direction of the path segment
    segmentDirection = endPoint - startPoint
    # Calculate the rotation matrix to align the path segment with the x-axis
    theta = np.arctan2(segmentDirection[1], segmentDirection[0])
    rotationMatrix = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    # Calculate the range of the path segment
    rng = np.sqrt(segmentDirection[0]**2 + segmentDirection[1]**2)
    # Generate the path with at least 1000 waypoints
    if numWaypoints > 1000:
        xx = np.linspace(0, rng, numWaypoints)
        zz = np.linspace(0, segmentDirection[2], numWaypoints)-surveyAltitude
    else :
        xx = np.linspace(0, rng, 1000)
        zz = np.linspace(0, segmentDirection[2], 1000)-surveyAltitude
    # Select waypoints from the path
    indices = np.linspace(0, xx.shape[0]-1, numWaypoints).astype(int)
    xx = xx[indices]
    zz = zz[indices]
    # Calculate the y sweep
    yy = sweepAmplitude*np.sin((numSweeps*2*np.pi*xx)/rng)
    # Rotate waypoints by theta to align with path segment
    waypoints = np.matmul(rotationMatrix,np.array([xx, yy, zz])).transpose()
    # Translate waypoints to path segment origin
    waypoints = waypoints + startPoint

    return waypoints

def POIPath(
        POIs: np.ndarray,
        POI_Labels: list,
        surveyAltitude: np.float32,
        sweepAmplitude: np.float32,
        numSweeps: np.float32,
        numWaypoints: np.uint8,
        plotFlag=False
        ):

    """
    Generate a path that visits a set of POIs.

    Args:
        POIs (numpy.ndarray): Array of POI coordinates.
        POI_Labels (list): List of POI labels.
        surveyAltitude (np.float32): Altitude at which to survey the POIs.
        sweepAmplitude (np.float32): Amplitude of the side-to-side sweep.
        numSweeps (np.float32): Number of side-to-side sweeps.
        numWaypoints (np.uint8): Number of waypoints per segment.
        plotFlag (bool, optional): Flag indicating whether to plot the path. Defaults to False.

    Returns:
        np.ndarray: Array of waypoints representing the survey path.
    """ 
    # Reset the POIs to the origin and convert to meters in the drone Frame
    dronePOIs = unreal2dronePOIs(POIs-POIs[0,:])
    # Compute the direction of each path segment between POIs
    segmentDirections = np.diff(dronePOIs, axis=0)
    # Initialize the path array
    numSegments = segmentDirections.shape[0]
    dronePath = np.zeros((numWaypoints, 3, numSegments))
    droneWaypoints = np.zeros((numWaypoints*numSegments, 3))
    # Compute the path between each POI and the next
    for segment in range(numSegments):
        # Compute the start and end points of the path segment
        startPoint = dronePOIs[segment,:]
        endPoint = dronePOIs[segment+1,:]
        # Compute the path between the start and end points
        waypoints = p2pSweep(
            startPoint=startPoint, 
            endPoint=endPoint, 
            surveyAltitude=surveyAltitude, 
            sweepAmplitude=sweepAmplitude, 
            numSweeps=numSweeps, 
            numWaypoints=numWaypoints
        )
        droneWaypoints[segment*numWaypoints:(segment+1)*numWaypoints,:] = waypoints

    if plotFlag:
        poiPath = np.concatenate(
            (
                dronePOIs[0,:].reshape(1,3),
                droneWaypoints,
                dronePOIs[-1,:].reshape(1,3),
            )
        )

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(poiPath[:,0], poiPath[:,1], poiPath[:,2], c='b', marker='.')
        ax.scatter(dronePOIs[:,0], dronePOIs[:,1], dronePOIs[:,2], c='r', marker='o')
        for i, text in enumerate(POI_Labels):
            ax.text(dronePOIs[i,0], dronePOIs[i,1], dronePOIs[i,2], text)
        ax.azim = -120
        ax.elev = -120
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect([1,1,1])
        ax.set_title('POI Survey Path in Drone Frame')
        plt.show()

    return droneWaypoints

def flyWaypoints(
        waypoints: np.ndarray, 
        playerSpeed: np.float32
    ):
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

    ic(client.getMultirotorState(vehicle_name='SimpleFlight'))

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

def pullFrames(
        numFrames: np.uint8, 
        frameRate: np.float32, 
        saveFolder: str
    ):
    """
    Pulls a specified number of frames from the AirSim simulator and saves them to the specified folder.

    Args:
        numFrames (int): Number of frames to pull.
        frameRate (np.float32): Frame rate in frames per second. Defaults to 10.0 fps.
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
    timeInterval = 1.0/frameRate

    time.sleep(5)
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
        airsim.write_file(os.path.join(povFolder, f"pov_0{imageIndex+i}.png"), responses[0].image_data_uint8)
        airsim.write_file(os.path.join(depthFolder, f"depth_0{imageIndex+i}.png"), responses[1].image_data_uint8)
        airsim.write_file(os.path.join(maskFolder, f"mask_0{imageIndex+i}.png"), responses[2].image_data_uint8)

        # Print the saved image names and folders
        print(f"Saved Image: {pov_img_name  } to {povFolder}")
        print(f"Saved Image: {depth_img_name} to {depthFolder}")
        print(f"Saved Image: {mask_img_name } to {maskFolder} \n")

        # Wait for the specified time interval before getting the next set of images
        time.sleep(timeInterval)

    return

def droneSpawn(
        waypoints: np.ndarray, 
        numDrones: np.uint8, 
        FOV: np.array, 
        plotFlag=False
    ):
    """
    Generates spawn points for a specified number of drones at each waypoint within the chief drones FOV.

    Args:
        waypoints (numpy.ndarray): Array of waypoints representing the flight path.
        numDrones (np.uint8): Number of drones to spawn at each waypoint.
        FOV (np.array): Field of view of the chief drone in the format [range, theta, phi].
        plotFlag (bool, optional): Flag indicating whether to plot the spawn points. Defaults to False.

    Returns:
        np.ndarray: Array of spawn points for each drone at each waypoint shape (numDrones,3,numWaypoints).
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
        thetas = tangentTheta + -thetaRange/2 + (np.random.rand(numDrones)*thetaRange)
        # Generate random Phi Angles centered around the tangent vector
        tangentPhi = np.arctan2(tangentVectors[wp, 2], np.sqrt(tangentVectors[wp, 0]**2 + tangentVectors[wp, 1]**2)) 
        phis = np.pi / 2 + tangentPhi + -phiRange/2 + (np.random.rand(numDrones)*phiRange)
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
        ax.set_aspect('equal')
        
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

def getPathTangents(
        waypoints: np.ndarray
    ):
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

    # Normalize nonZero Tangent Vectors
    for i in range(tangentVectors.shape[0]):
        if np.linalg.norm(tangentVectors[i,:]) > 0:
            tangentVectors[i,:] = tangentVectors[i,:]/np.linalg.norm(tangentVectors[i,:])
        else:
            tangentVectors[i,:] = tangentVectors[i-1,:]/np.linalg.norm(tangentVectors[i-1,:])

    return tangentVectors

def spherical2cartesian(
        r: float, 
        theta: float, 
        phi: float,
    ):
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

def plotFOVs(
        waypoints: np.ndarray, 
        spawnpoints: np.ndarray, 
        FOV: np.array, 
        waypointIndex: np.uint8, 
        fig=None
    ):
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
        ax.set_aspect('equal')
        plt.show()
    return
