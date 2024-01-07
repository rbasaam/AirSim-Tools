# AirSim Tools

This repository contains a set of tools for working with the AirSim simulator. The tools are implemented in Python and utilize the AirSim API.

## Installation

Clone the repository:

```bash
git clone https://github.com/rbasaam/AirSim-Tools.git
```

Install the required dependencies:

- [AirSim](https://github.com/AVL-TMU/UnrealEngine-AirSim-PX4.git)
- numpy
- matplotlib

## AirSim Settings JSON

The following settings are used for AirSim 
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "DisplayMode": "FlightCamera",
  "Vehicles" : {
      "chiefDrone" : {
          "VehicleType" : "SimpleFlight",
          "AutoCreate" : true,
          "Cameras" : {
              "highResolution": {
                  "CaptureSettings" : [
                      {
                          "ImageType" : 0,
                          "Width" : 4320,
                          "Height" : 2160
                      }
                  ],
                  "X": 0.50, "Y": 0.00, "Z": 0.10,
                  "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
              },
              "lowResolution": {
                  "CaptureSettings" : [
                      {
                          "ImageType" : 0,
                          "Width" : 256,
                          "Height" : 144
                      }
                  ],
                  "X": 0.50, "Y": 0.00, "Z": 0.10,
                  "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
              }
          }
      }
  }
}
```

## Image Acquisition

Different Cameras can be initialized in the `settings.json` file

Available ImageType Values:

    Scene = 0, 
    DepthPlanar = 1, 
    DepthPerspective = 2,
    DepthVis = 3, 
    DisparityNormalized = 4,
    Segmentation = 5,
    SurfaceNormals = 6,
    Infrared = 7,
    OpticalFlow = 8,
    OpticalFlowVis = 9


