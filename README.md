# Send MultiRotor On Custom Flight Path

```customFlightPath.py``` allows for a user to define waypoints along a flight path of their own design. Note that an increase in altitude corresponds with a decrease in the z coordinate axis (-z UP).

To Run, open a cmd terminal while AirSim is open and running and run the following command,

```cmd
python customFlightPath.py
```

# Pull Image Frames from AirSim
```dataRecorder.py``` is a simple script that when run through the cmd terminal will pull the Scene, Depth and Semantically Segmented image frames at a defined time interval until it saves a number of frames specified in the script

