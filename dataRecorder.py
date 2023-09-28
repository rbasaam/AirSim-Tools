import airsim
import time
import os

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Specify the folders to save the images
saveFolder = "saved_imgs/"
depthFolder = "saved_imgs/depth"
povFolder = "saved_imgs/pov"
maskFolder = "saved_imgs/mask"

# Ensure the folders exist
os.makedirs(saveFolder, exist_ok=True)
os.makedirs(depthFolder, exist_ok=True)
os.makedirs(povFolder, exist_ok=True)
os.makedirs(maskFolder, exist_ok=True)

# Specify the number of images to save and the time interval between images
numImages = 3
timeInterval = 2  # in seconds

for i in range(numImages):
    
    # Specify the image names
    pov_img_name   = f"pov_{i}.png"
    depth_img_name = f"depth_{i}.png"
    mask_img_name  = f"mask_{i}.png"

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