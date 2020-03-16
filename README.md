# explorer_robot
implementation of a package for exploration and mapping

# Package required

## Make sure the simulation package is installed:

sudo apt-get install ros-<distro>-husky-simulator

## Set an environmental variable HUSKY_GAZEBO_DESCRIPTION:

export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

# Run

## bring up the robot by command
$ roslaunch explorer_turtle turtlebot_start.launch
## We bring up 3D node, gmapping node, move base node
$ roslaunch turtlebot_gazebo gmapping_demo.launch
## run map create
$ roslaunch explorer_turtle map.launch
## teleop
$ roslaunch turtlebot_teleop keyboard_teleop.launch

#Code

## Extended Kalman Filter

Based on available information (control inputs and observations) it is required to obtain an estimate of the Turtlebot's state that optimizes a given criteria. This is the role played by the Extended Kalman Filter (EKF).

### Initialization

The first step is to initialize our filter based on prior knowledge of the state. We assume that we know exactly where the Turtlebot is in the world at the start of measurement, so we initialize everything to zero. 

```
# PRIOR KNOWLEDGE OF STATE
predicted_state_est = 0
predicted_covariance_est = 0
state_trans_uncertainty_noise = 0
measurement =	0
ave_meas_dist = 0

```
### Prediction Step

The next step is to make a prediction about where the Turtlebot currently is based on our previous knowledge of the state and the control commands given to the Turtlebot wheels. This is easy to obtain via the /odom topic published by the base nodelet manager. 

1. Create a subscriber that listens to the /odom topic

```
rospy.Subscriber('odom', Odometry, odom_state_prediction)

```
2. The odometry data published is a standard message with 7 configuration variables: x, y , z and quaternions.The next step is to extract the three configuration variables that describe our system: x, y, yaw (our model's configuration).

We can use the tf.transforms euler_from_quaternion function in order to get the yaw in eu 

```
	# Get the yaw (theta) from the quaternion using the tf.transformations euler_from_quaternion function. 
	(roll,pitch,yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
	x = odom_data.pose.pose.position.x
	y = odom_data.pose.pose.position.y

```

3. The final part of the prediction step is to quantize the error in our prediction: 

```
# UNCERTAINTY INTRODUCED BY STATE TRANSITION (MEAN = 0, COVARIANCE PUBLISHED BY ODOM TOPIC: )
	# Odom covariance matrix is 6 x 6. We need a 3 x 3 covariance matrix of x, y and theta. Omit z, roll and pitch data. 
	state_trans_uncertainty_noise= numpy.array([[odom_data.pose.covariance[0],odom_data.pose.covariance[1],odom_data.pose.covariance[5]],[odom_data.pose.covariance[6],odom_data.pose.covariance[7],odom_data.pose.covariance[11]], [odom_data.pose.covariance[30],odom_data.pose.covariance[31],odom_data.pose.covariance[35]]])
	
	# Define the state transition jacobian. 
	state_transition_jacobian = numpy.array([[1,0,0],[0,1,0],[0,0,1]])
	
	# Calculated the total uncertainty of the predicted state estimate
	predicted_covariance_est = state_transition_jacobian*predicted_covariance_est*numpy.transpose(state_transition_jacobian)+state_trans_uncertainty_noise

```

### Measurement Update Step

After a prediction has been made on the current pose, we use measurement information to update our prediction. Our simple model uses kinect scan data to measure the distance of the Turtlebot from the wall. 

1. Subscribe to the /scan topic published by the pointcloud_to_laserscan package: 

```
rospy.Subscriber('scan', LaserScan, kinect_scan_estimate)

```

2. For simplicity, we average 20 measured distances in the middle of the sensor range (pointing directly in front of the Turtlebot's vision) .

```
# Average measured range/distance of 20 samples from sample 310 to 330 (out of a total 640 samples)
	sum_dist = 0
	length = 0
	#for i in range (310, 330):
	for i in range (580, 600):
		if str(measurement[i]) != 'nan' :
			sum_dist += measurement[i]
			length += 1 

	if length != 0: 
		ave_meas_dist = sum_dist/length

```

3. The next step is to find the difference between our prediction and the measurement information. In order to do so, we must calculate the measurement that we expect to get if our prediction is accurate. This expected measurement is subtracted from the actual measurement to give us a residual.  

```
# Calculating the measurement we expect to get based on received odometery data. 
	expected_meas = numpy.cross(numpy.array([0, 1, 0]), numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]))
	
	#innovation or measurement residual: The difference between our actual measurement and the measurement we expected to get. 
	meas_residual = ave_meas_dist - expected_meas

```

4. We also find the uncertainty inherent in our residual calculation: 

```
#innovation or measurement residual: The difference between our actual measurement and the measurement we expected to get. 
	meas_residual = ave_meas_dist - expected_meas

	#Account for the measurement noise by adding error 
	meas_noise_covariance = 0.005

	# Measurement jacobian: 
	H = numpy.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

	# Innovation (or residual) covariance
	residual_covariance = H*predicted_covariance_est*numpy.transpose(H)+meas_noise_covariance

```

5. We now have enough information to find the best estimate of the state using our knowledge of the system model, measurements, and including uncertainties: 

```
# Near-optimal Kalman gain: Amalgamating all the uncertainties
	kalman_gain = predicted_covariance_est*numpy.transpose(H)*numpy.linalg.inv(residual_covariance)

	# Updated state estimate
	updated_state_estimate =  numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]) + numpy.dot(kalman_gain, meas_residual)


```

6. Finally, we calculate the uncertainty of our final best estimate: 

```
# Updated covariance estimate
	predicted_covariance_est= (numpy.identity(3) - numpy.cross(kalman_gain,H))*predicted_covariance_est

```

## Reference Provider

Every time the controller requests a pose, the reference_provider will respond. For now, the response of the reference_provider is a Config variable where the x and theta are the same as the current robot pose, and the y coordinate is incremented by 1 mm, so the robot moves in a straight line along the wall. 

```
# Only move in the y direction in 1 mm increments. 
	x_desired = x_now
	y_desired = y_now + 0.1
	th_desired = th_now

	# Package desired state in a custom message of type 'Config' 
	desired_state = Config(x_desired, y_desired, th_

```
