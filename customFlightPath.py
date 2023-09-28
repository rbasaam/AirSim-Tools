import airsim

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


# record the starting position
start_pos = client.simGetVehiclePose().position

# define hover altitude and flight speed
z = 30
speed = 7

# define waypoints
waypoints = [
    airsim.Vector3r(0,0, -z - 1),
    airsim.Vector3r(-2.6323,-2.521, -z - 0.99903),
    airsim.Vector3r(-5.2189,-5.042, -z - 0.99613),
    airsim.Vector3r(-7.7146,-7.563, -z - 0.9913),
    airsim.Vector3r(-10.0761,-10.084, -z - 0.98455),
    airsim.Vector3r(-12.2623,-12.605, -z - 0.9759),
    airsim.Vector3r(-14.2352,-15.1261, -z - 0.96535),
    airsim.Vector3r(-15.9603,-17.6471, -z - 0.95294),
    airsim.Vector3r(-17.4078,-20.1681, -z - 0.93869),
    airsim.Vector3r(-18.5525,-22.6891, -z - 0.92261),
    airsim.Vector3r(-19.3743,-25.2101, -z - 0.90475),
    airsim.Vector3r(-19.859,-27.7311, -z - 0.88514),
    airsim.Vector3r(-19.9983,-30.2521, -z - 0.86382),
    airsim.Vector3r(-19.7895,-32.7731, -z - 0.84082),
    airsim.Vector3r(-19.2365,-35.2941, -z - 0.8162),
    airsim.Vector3r(-18.3488,-37.8151, -z - 0.78999),
    airsim.Vector3r(-17.1418,-40.3361, -z - 0.76226),
    airsim.Vector3r(-15.6366,-42.8571, -z - 0.73305),
    airsim.Vector3r(-13.8594,-45.3782, -z - 0.70242),
    airsim.Vector3r(-11.841,-47.8992, -z - 0.67044),
    airsim.Vector3r(-9.6166,-50.4202, -z - 0.63715),
    airsim.Vector3r(-7.2248,-52.9412, -z - 0.60263),
    airsim.Vector3r(-4.7074,-55.4622, -z - 0.56695),
    airsim.Vector3r(-2.1081,-57.9832, -z - 0.53017),
    airsim.Vector3r(0.52794,-60.5042, -z - 0.49236),
    airsim.Vector3r(3.1548,-63.0252, -z - 0.4536),
    airsim.Vector3r(5.7267,-65.5462, -z - 0.41396),
    airsim.Vector3r(8.199,-68.0672, -z - 0.37352),
    airsim.Vector3r(10.5286,-70.5882, -z - 0.33235),
    airsim.Vector3r(12.6751,-73.1092, -z - 0.29055),
    airsim.Vector3r(14.601,-75.6303, -z - 0.24818),
    airsim.Vector3r(16.2729,-78.1513, -z - 0.20533),
    airsim.Vector3r(17.6617,-80.6723, -z - 0.16208),
    airsim.Vector3r(18.7432,-83.1933, -z - 0.11852),
    airsim.Vector3r(19.4986,-85.7143, -z - 0.07473),
    airsim.Vector3r(19.9147,-88.2353, -z - 0.030795),
    airsim.Vector3r(19.9843,-90.7563, -z - -0.0132),
    airsim.Vector3r(19.7063,-93.2773, -z - -0.057169),
    airsim.Vector3r(19.0853,-95.7983, -z - -0.10103),
    airsim.Vector3r(18.1323,-98.3193, -z - -0.14469),
    airsim.Vector3r(16.8639,-100.8403, -z - -0.18807),
    airsim.Vector3r(15.302,-103.3613, -z - -0.23109),
    airsim.Vector3r(13.4739,-105.8824, -z - -0.27366),
    airsim.Vector3r(11.4114,-108.4034, -z - -0.3157),
    airsim.Vector3r(9.1503,-110.9244, -z - -0.35714),
    airsim.Vector3r(6.73,-113.4454, -z - -0.39787),
    airsim.Vector3r(4.1927,-115.9664, -z - -0.43784),
    airsim.Vector3r(1.5823,-118.4874, -z - -0.47697),
    airsim.Vector3r(-1.0555,-121.0084, -z - -0.51516),
    airsim.Vector3r(-3.675,-123.5294, -z - -0.55236),
    airsim.Vector3r(-6.2305,-126.0504, -z - -0.5885),
    airsim.Vector3r(-8.6777,-128.5714, -z - -0.62349),
    airsim.Vector3r(-10.9738,-131.0924, -z - -0.65728),
    airsim.Vector3r(-13.0791,-133.6134, -z - -0.68979),
    airsim.Vector3r(-14.9567,-136.1345, -z - -0.72097),
    airsim.Vector3r(-16.5742,-138.6555, -z - -0.75075),
    airsim.Vector3r(-17.9033,-141.1765, -z - -0.77908),
    airsim.Vector3r(-18.9209,-143.6975, -z - -0.8059),
    airsim.Vector3r(-19.6092,-146.2185, -z - -0.83116),
    airsim.Vector3r(-19.9565,-148.7395, -z - -0.85482),
    airsim.Vector3r(-19.9565,-151.2605, -z - -0.87681),
    airsim.Vector3r(-19.6092,-153.7815, -z - -0.89712),
    airsim.Vector3r(-18.9209,-156.3025, -z - -0.91568),
    airsim.Vector3r(-17.9033,-158.8235, -z - -0.93247),
    airsim.Vector3r(-16.5742,-161.3445, -z - -0.94746),
    airsim.Vector3r(-14.9567,-163.8655, -z - -0.96061),
    airsim.Vector3r(-13.0791,-166.3866, -z - -0.97191),
    airsim.Vector3r(-10.9738,-168.9076, -z - -0.98132),
    airsim.Vector3r(-8.6777,-171.4286, -z - -0.98883),
    airsim.Vector3r(-6.2305,-173.9496, -z - -0.99443),
    airsim.Vector3r(-3.675,-176.4706, -z - -0.9981),
    airsim.Vector3r(-1.0555,-178.9916, -z - -0.99985),
    airsim.Vector3r(1.5823,-181.5126, -z - -0.99965),
    airsim.Vector3r(4.1927,-184.0336, -z - -0.99752),
    airsim.Vector3r(6.73,-186.5546, -z - -0.99346),
    airsim.Vector3r(9.1503,-189.0756, -z - -0.98748),
    airsim.Vector3r(11.4114,-191.5966, -z - -0.97959),
    airsim.Vector3r(13.4739,-194.1176, -z - -0.9698),
    airsim.Vector3r(15.302,-196.6387, -z - -0.95813),
    airsim.Vector3r(16.8639,-199.1597, -z - -0.94461),
    airsim.Vector3r(18.1323,-201.6807, -z - -0.92926),
    airsim.Vector3r(19.0853,-204.2017, -z - -0.91211),
    airsim.Vector3r(19.7063,-206.7227, -z - -0.89319),
    airsim.Vector3r(19.9843,-209.2437, -z - -0.87255),
    airsim.Vector3r(19.9147,-211.7647, -z - -0.85022),
    airsim.Vector3r(19.4986,-214.2857, -z - -0.82624),
    airsim.Vector3r(18.7432,-216.8067, -z - -0.80066),
    airsim.Vector3r(17.6617,-219.3277, -z - -0.77353),
    airsim.Vector3r(16.2729,-221.8487, -z - -0.74491),
    airsim.Vector3r(14.601,-224.3697, -z - -0.71484),
    airsim.Vector3r(12.6751,-226.8908, -z - -0.68339),
    airsim.Vector3r(10.5286,-229.4118, -z - -0.65062),
    airsim.Vector3r(8.199,-231.9328, -z - -0.61659),
    airsim.Vector3r(5.7267,-234.4538, -z - -0.58136),
    airsim.Vector3r(3.1548,-236.9748, -z - -0.54501),
    airsim.Vector3r(0.52794,-239.4958, -z - -0.5076),
    airsim.Vector3r(-2.1081,-242.0168, -z - -0.46921),
    airsim.Vector3r(-4.7074,-244.5378, -z - -0.42992),
    airsim.Vector3r(-7.2248,-247.0588, -z - -0.38979),
    airsim.Vector3r(-9.6166,-249.5798, -z - -0.3489),
    airsim.Vector3r(-11.841,-252.1008, -z - -0.30734),
    airsim.Vector3r(-13.8594,-254.6218, -z - -0.26519),
    airsim.Vector3r(-15.6366,-257.1429, -z - -0.22252),
    airsim.Vector3r(-17.1418,-259.6639, -z - -0.17942),
    airsim.Vector3r(-18.3488,-262.1849, -z - -0.13598),
    airsim.Vector3r(-19.2365,-264.7059, -z - -0.092268),
    airsim.Vector3r(-19.7895,-267.2269, -z - -0.048381),
    airsim.Vector3r(-19.9983,-269.7479, -z - -0.0044),
    airsim.Vector3r(-19.859,-272.2689, -z - 0.03959),
    airsim.Vector3r(-19.3743,-274.7899, -z - 0.083502),
    airsim.Vector3r(-18.5525,-277.3109, -z - 0.12725),
    airsim.Vector3r(-17.4078,-279.8319, -z - 0.17076),
    airsim.Vector3r(-15.9603,-282.3529, -z - 0.21393),
    airsim.Vector3r(-14.2352,-284.8739, -z - 0.25669),
    airsim.Vector3r(-12.2623,-287.395, -z - 0.29896),
    airsim.Vector3r(-10.0761,-289.916, -z - 0.34064),
    airsim.Vector3r(-7.7146,-292.437, -z - 0.38167),
    airsim.Vector3r(-5.2189,-294.958, -z - 0.42195),
    airsim.Vector3r(-2.6323,-297.479, -z - 0.46142),
    airsim.Vector3r(0,-300, -z -0.5)
]

# takeoff
client.takeoffAsync().join()

# fly along the waypoints
client.moveOnPathAsync(waypoints, speed, 120, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True,3), -1, 1).join()

# return to starting position at a hover altitude defined by z
client.moveToPositionAsync(start_pos.x_val, start_pos.y_val, -z, speed).join()

# land
client.landAsync().join()