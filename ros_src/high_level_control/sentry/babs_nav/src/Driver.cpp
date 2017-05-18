#include "Driver.h"

void mySigintHandler(int sig)
{
	ROS_WARN("mySigintHandler called");
	close(sockfd); // be sure the close the socket on shutdown
	ros::shutdown();
	ROS_WARN("mySigintHandler done");
}

// constructor
Driver::Driver(ros::NodeHandle * nodehandle) : n(*nodehandle)
{
	prediction_memory = 0.5; // init to unbiased value
	shouldProcess     = true;
	imuReady            = false;
	contactFailure      = false;
	merging_cloud       = false;
	firstCloudSeen      = false;
	firstCloudProcessed = false;
	fileCounter         = 0;
	des_state_speed     = TARGET_SPEED;
	des_state_omega     = 0.0;
	// hard code a simple path: the world x axis
	des_state_x    = 0.0;
	des_state_y    = 0.0;
	des_state_psi  = 0.0; // M_PI/2;
	half_sec       = ros::Duration(0.5);
	five_ms        = ros::Duration(0.005);
	one_ms         = ros::Duration(0.001);
	imu_subscriber = n.subscribe("/imu_quat", 1, &Driver::imuCallback, this);
	ros::spinOnce();
	while (!imuReady) {
		ROS_INFO("waiting for imu");
		ros::spinOnce();
		half_sec.sleep();
	}

	kinectcloud  = *new sensor_msgs::PointCloud2();
	kinectcloud1 = *new sensor_msgs::PointCloud2();
	kinectcloud2 = *new sensor_msgs::PointCloud2();
	kinectcloud3 = *new sensor_msgs::PointCloud2();

	voxel_cloud_pub    = n.advertise<sensor_msgs::PointCloud2>("/voxel_cloud", 1);
	cropped_cloud_pub1 = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud1", 1);
	cropped_cloud_pub2 = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud2", 1);
	cropped_cloud_pub3 = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud3", 1);
	cropped_cloud_pub4 = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud4", 1);
	cmd_publisher      = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

	pcl_subscriber  = n.subscribe("/kinect/depth/points", 1, &Driver::pointCloudCallback, this);
	pcl_subscriber1 = n.subscribe("/kinect1/depth/points1", 1, &Driver::pointCloudCallback1, this);
	pcl_subscriber2 = n.subscribe("/kinect2/depth/points2", 1, &Driver::pointCloudCallback2, this);
	pcl_subscriber3 = n.subscribe("/kinect3/depth/points3", 1, &Driver::pointCloudCallback3, this);

	gms_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getmodelstate.request.model_name = "robot_description";
	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/set_model_state", true);
		ROS_INFO("waiting for set_model_state service");
		half_sec.sleep();
	}
	ROS_INFO("set_model_state service exists");
	set_model_state_client =
	  n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ROS_INFO("setting up socket connection to server");
	int portno, n;
	half_sec.sleep();
	struct sockaddr_in serv_addr;
	struct hostent * server;
	portno = 51717;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		ROS_WARN("ERROR opening socket");
	server = gethostbyname("127.0.0.1");
	if (server == NULL) {
		fprintf(stderr, "ERROR, no such host\n");
		exit(0);
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *) server->h_addr, (char *) &serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		ROS_WARN("ERROR connecting");
	// Override the default ros sigint handler.
	signal(SIGINT, mySigintHandler);
}

void Driver::process_cloud(const sensor_msgs::PointCloud2& input)
{
	// FIRST downsample cloud using a voxelgrid, reduces number of points we need
	// to process from 1,228,800 to less than 90,000
	// sensor_msgs::PointCloud2 voxel_out;
	// voxelFilter(input, voxel_out);

	// get the most updated location
	gms_client.call(getmodelstate);
	shouldProcess = false;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// pcl::fromROSMsg(voxel_out, cloud);
	pcl::fromROSMsg(input, cloud);

	tf::Quaternion q(getmodelstate.response.pose.orientation.x,
	  getmodelstate.response.pose.orientation.y,
	  getmodelstate.response.pose.orientation.z,
	  getmodelstate.response.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	// https://en.wikipedia.org/wiki/Rotation_matrix
	Eigen::Matrix<double, 4, 4> tmp;
	double theta = -pitch;
	// rotation about the x axis
	tmp(0, 0) = 1;
	tmp(0, 1) = 0;
	tmp(0, 2) = 0;
	tmp(0, 3) = 0; // x axis translation
	tmp(1, 0) = 0;
	tmp(1, 1) = cos(theta);
	tmp(1, 2) = -sin(theta);
	tmp(1, 3) = 0; // y axis translationtake_snapshot
	tmp(2, 1) = sin(theta);
	tmp(2, 2) = cos(theta); // https://en.wikipedia.org/wiki/Rotation_matrix
	tmp(2, 3) = 0;          // z axis translation
	tmp(3, 0) = 0;
	tmp(3, 1) = 0;
	tmp(3, 2) = 0;
	tmp(3, 3) = 1;
	const Eigen::Matrix<double, 4, 4> xtransform(tmp);
	theta = roll;
	// rotation about the z axis
	tmp(0, 0) = cos(theta);
	tmp(0, 1) = -sin(theta);
	tmp(0, 2) = 0;
	tmp(0, 3) = 0; // x axis translation
	tmp(1, 0) = sin(theta);
	tmp(1, 1) = cos(theta);
	tmp(1, 2) = 0;
	tmp(1, 3) = 0; // y axis translationtmp
	tmp(2, 3) = 0; // z axis translation
	tmp(3, 0) = 0;
	tmp(3, 1) = 0;
	tmp(3, 2) = 0;
	tmp(3, 3) = 1;
	const Eigen::Matrix<double, 4, 4> ztransform(tmp);
	// VERY IMPORTANT to do roll transform then pitch transform because the data's from a quaternion!
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	pcl::transformPointCloud(cloud, transformed_cloud, ztransform);
	cloud = transformed_cloud;
	pcl::transformPointCloud(cloud, transformed_cloud, xtransform);
	cloud = transformed_cloud;
	int cloudsize = (cloud.width) * (cloud.height);

	// ROS_WARN("cloudsize w h is: %d %d %d", cloudsize, cloud.width, cloud.height);

	// we're interesting only in the 1m x 1m square in front of the robot
	// double xMin = -0.5;
	// double xMax = 0.5;
	// double yMin = 0.5;
	// double yMax = 1.5;

	// and we need to limit the height range
	// height range should be be centered on ground instead of the height of the wobbler,
	// wobbler is about 1 meter of the ground (but wobbler height is considered height 0)
	double zMin = -5;
	double zMax = 3;
	int xdim    = (int) (targetWidth / resolution);
	int ydim    = (int) (targetLen / resolution);

	double rectangleMatrix[NUM_AREAS][6];
	double intensityMatrix[NUM_AREAS][xdim * ydim];

	pcl::PointCloud<pcl::PointXYZ> * pcdArray = new pcl::PointCloud<pcl::PointXYZ> [NUM_AREAS];

	double angle_delta = 2 * M_PI / (NUM_AREAS / 2); // so we end up with a circle of 8 rectangles
	for (int j = 0; j < NUM_AREAS; j++) {
		double theta = angle_delta * j; // starts at 90 degrees, aka right in front of the robot
		// defined as the middle point of the front edge of the area of interest

		double extra_spacing = 0.4;
		double rectStartX;
		double rectStartY;

		if (j < NUM_AREAS / 2) {
			rectStartX = cos(theta) * (frontSpacing + bodyLen / 2 + extra_spacing);
			rectStartY = sin(theta) * (frontSpacing + bodyLen / 2 + extra_spacing) - bodyLen / 2;
		} else {
			rectStartX = cos(theta) * (frontSpacing + bodyLen / 2 + targetLen + extra_spacing);
			rectStartY = sin(theta) * (frontSpacing + bodyLen / 2 + targetLen + extra_spacing) - bodyLen / 2;
		}
		double dx = targetWidth / 2 * sin(M_PI - theta);
		double dy = targetWidth / 2 * cos(M_PI - theta);

		// counts going counter clockwise starting at the front left point
		double rectangleX0 = rectStartX - dx;
		double rectangleY0 = rectStartY - dy;
		double rectangleX1 = rectStartX + dx;
		double rectangleY1 = rectStartY + dy;

		double d2x = targetLen * cos(M_PI - theta);
		double d2y = targetLen * sin(M_PI - theta);

		double rectangleX3 = rectangleX0 - d2x;
		double rectangleY3 = rectangleY0 + d2y;

		// ROS_INFO("rectStart: %f, %f ", rectStartX, rectStartY);
		// ROS_INFO("rect points %f, %f, %f, %f, %f, %f ", rectangleX0, rectangleY0,
		//   rectangleX1, rectangleY1, rectangleX3, rectangleY3);
		// ROS_INFO("----------------------------------");

		rectangleMatrix[j][0] = rectangleX0;
		rectangleMatrix[j][1] = rectangleY0;
		rectangleMatrix[j][2] = rectangleX1;
		rectangleMatrix[j][3] = rectangleY1;
		rectangleMatrix[j][4] = rectangleX3;
		rectangleMatrix[j][5] = rectangleY3;

		// clone the whole cloud
		pcdArray[j] = pcl::PointCloud<pcl::PointXYZ>(cloud);
		// pcdArray[j] = *cloud.makeShared();

		// initialize with nan
		for (int i = 0; i < xdim * ydim; i++) {
			intensityMatrix[j][i] = NAN;
		}
	}
	// clone the cloud, will fill with NAN's so we have an empty cloud for later
	// pcl::PointCloud<pcl::PointXYZ> emptyCloud = pcl::PointCloud<pcl::PointXYZ>(cloud);
	pcl::PointCloud<pcl::PointXYZ> emptyCloud = pcl::PointCloud<pcl::PointXYZ>(cloud);

	// loop through cloud, cropping out each desired image area
	for (int i = 0; i < cloudsize; i++) {
		emptyCloud[i].x = NAN;
		emptyCloud[i].y = NAN;
		emptyCloud[i].z = NAN;
		// in the gazebo's frame the point cloud's y is in the -z direction,
		// z is in the y direction, and x is the same
		double pointX = cloud[i].x;
		double pointY = cloud[i].z;
		double pointZ = -cloud[i].y;

		for (int j = 0; j < NUM_AREAS; j++) {
			double rectangleX0 = rectangleMatrix[j][0];
			double rectangleY0 = rectangleMatrix[j][1];
			double rectangleX1 = rectangleMatrix[j][2];
			double rectangleY1 = rectangleMatrix[j][3];
			double rectangleX3 = rectangleMatrix[j][4];
			double rectangleY3 = rectangleMatrix[j][5];

			// only care about points that fall in our square
			double x1    = pointX - rectangleX0;
			double y1    = pointY - rectangleY0;
			double x2    = rectangleX1 - rectangleX0;
			double y2    = rectangleY1 - rectangleY0;
			double dist1 = sqrt(x1 * x1 + y1 * y1);
			double dist2 = sqrt(x2 * x2 + y2 * y2);
			// calculate the angle between the point, the rectangle origin, and the
			// opposite corner of the rectangle (along the x axis)
			double angle = acos((x1 * x2 + y1 * y2) / (dist1 * dist2));
			double dx    = dist1 * cos(angle);
			// double dy       = dist1 * sin(angle);
			double fraction = dx / targetWidth;
			// round by adding 0.5 and casting (truncating)
			int xIndex = (int) ((xdim - 1) * fraction + 0.5);

			x1    = rectangleX3 - rectangleX0;
			y1    = rectangleY3 - rectangleY0;
			x2    = pointX - rectangleX0;
			y2    = pointY - rectangleY0;
			dist1 = sqrt(x1 * x1 + y1 * y1);
			dist2 = sqrt(x2 * x2 + y2 * y2);
			angle = acos((x1 * x2 + y1 * y2) / (dist1 * dist2));
			double dy = dist2 * cos(angle);

			fraction = dy / targetLen;
			int yIndex = (int) ((ydim - 1) * fraction + 0.5);

			// ROS_WARN("xIndex, yIndex: %d, %d", xIndex, yIndex);

			// Check if point falls within our rectangular area of interest.
			if (xIndex >= 0 && yIndex >= 0 && xIndex < xdim && yIndex < ydim) {
				int index = xIndex + yIndex * xdim;
				fraction = (pointZ - zMin) / (zMax - zMin);
				if (fraction > 1) {
					fraction = 1;
				} else if (fraction < 0) {
					fraction = 0;
				}
				// scale fall in range 0-199
				// 255 is reserved for unknown pixels, the unused gap 200-254 may help the
				// future neural net more easily differentiate unknowns from other values
				double value = fraction * 199;
				// save the pixel as the highest point we come across
				if (isnan(intensityMatrix[j][index]) || intensityMatrix[j][index] < value) {
					intensityMatrix[j][index] = value;
				}
			} else {
				// otherwise we don't want these points, set them to NAN
				pcdArray[j][i].x = NAN;
				pcdArray[j][i].z = NAN;
				pcdArray[j][i].y = NAN;
			}
		}
	}

	sensor_msgs::PointCloud2 emptyCloud2;
	pcl::toROSMsg(emptyCloud, emptyCloud2);
	sensor_msgs::PointCloud2 * merged1 = new sensor_msgs::PointCloud2();
	sensor_msgs::PointCloud2 * merged2 = new sensor_msgs::PointCloud2();
	sensor_msgs::PointCloud2 * merged3 = new sensor_msgs::PointCloud2();
	sensor_msgs::PointCloud2 * merged4 = new sensor_msgs::PointCloud2();
	bool merged1hit = false;
	bool merged2hit = false;
	bool merged3hit = false;
	bool merged4hit = false;

	for (int j = 0; j < NUM_AREAS; j++) {
		int imageMatrix[xdim][ydim];
		for (unsigned int xIndex = 0; xIndex < xdim; xIndex++) {
			for (unsigned int yIndex = 0; yIndex < ydim; yIndex++) {
				int index = xIndex + yIndex * xdim;
				if (isnan(intensityMatrix[j][index])) {
					// anything still nan is unknown value so set it to 255 to
					// separate it from the 0-200 range
					imageMatrix[xIndex][yIndex] = 255;
				} else {
					// add 0.5 to round instead of truncating
					imageMatrix[xIndex][yIndex] = (int) (intensityMatrix[j][index] + 0.5);
				}
			}
		}

		// classify imagematrix, really tensorflow should be natively run in c++ here
		// but their c++ compatibility is trash right now so I'm using this python server instead.
		ostringstream os;
		os << "[";
		// First, loop through converting to a string for transport
		for (int k = 0; k < NUM_IMAGES; k++) {
			os << "[";
			for (unsigned int j = 0; j < ydim; j++) {
				for (unsigned int i = 0; i < xdim; i++) {
					if (j + 1 == ydim && i + 1 == xdim) {
						// std::ostringstream ss;
						// ss << Number;
						// return ss.str();

						os << imageMatrix[i][j];
					} else {
						os << imageMatrix[i][j] << ",";
					}
				}
			}
			if (k + 1 == NUM_IMAGES)
				os << "]";
			else
				os << "],";
		}
		os << "]";
		string tmp2 = os.str();

		// ROS_INFO("image is: %s ", tmp2.c_str());
		// ROS_INFO("j is: %d ", j);

		int result = write(sockfd, tmp2.c_str(), strlen(tmp2.c_str()));
		if (result < 0)
			ROS_WARN("ERROR writing to socket");
		char prediction_buffer[256];
		bzero(prediction_buffer, 256);
		result = read(sockfd, prediction_buffer, 255);
		if (result < 0)
			ROS_WARN("ERROR reading from socket");
		string prediction_string = prediction_buffer;
		ostringstream num_stream;
		int k = 0;
		for (int i = 0; i < prediction_string.length(); i++) {
			if (prediction_string[i] != ' ') {
				num_stream << prediction_string[i];
			} else {
				string s   = num_stream.str();
				double num = atof(s.c_str());
				predictionMatrix[j][k] = num;
				num_stream.clear();
				num_stream.str("");
				k++;
			}
		}

		// ROS_WARN("CLASSIFICATION RESULT(S): ");
		// for (int i = 0; i < NUM_IMAGES; i++) {
		//  ROS_INFO("image%d ------------------------>   %f", i, predictions[i]);
		// }

		// publish the voxel cloud
		sensor_msgs::PointCloud2 voxel_cloud;
		pcl::toROSMsg(cloud, voxel_cloud);
		sensor_msgs::PointCloud2 croppedCloud;
		pcl::toROSMsg(pcdArray[j], croppedCloud);

		if (!voxel_cloud_pub || !cropped_cloud_pub1 || !cropped_cloud_pub2 || !cropped_cloud_pub3 ||
		  !cropped_cloud_pub4)
		{
			ROS_WARN("Publisher invalid!");
		} else {
			voxel_cloud_pub.publish(voxel_cloud);
			sensor_msgs::PointCloud2 * tmpMerged = new sensor_msgs::PointCloud2();
			// merge the 8 cropped clouds based on what class they fall into
			if (predictionMatrix[j][0] < 0.25) {
				// if (j == 0)
				//  ROS_WARN("DANGER! Will Robinson! Danger!!! --------> %f", predictionMatrix[j][0]);
				merged1hit = true;
				pcl::concatenatePointCloud(*merged1, croppedCloud, *tmpMerged);
				*merged1 = *tmpMerged;
			} else if (predictionMatrix[j][0] < 0.5) {
				// if (j == 0)
				//  ROS_WARN("Watch out -------------------------------> %f", predictionMatrix[j][0]);
				merged2hit = true;
				pcl::concatenatePointCloud(*merged2, croppedCloud, *tmpMerged);
				*merged2 = *tmpMerged;
			} else if (predictionMatrix[j][0] < 0.75) {
				// if (j == 0)
				//  ROS_INFO("Careful now -----------------------------> %f", predictionMatrix[j][0]);
				merged3hit = true;
				pcl::concatenatePointCloud(*merged3, croppedCloud, *tmpMerged);
				*merged3 = *tmpMerged;
			} else {
				// if (j == 0)
				//  ROS_INFO("Lookin good -----------------------------> %f", predictionMatrix[j][0]);
				merged4hit = true;
				pcl::concatenatePointCloud(*merged4, croppedCloud, *tmpMerged);
				*merged4 = *tmpMerged;
			}
			delete tmpMerged;
		}
	}

	if (merged4hit)
		cropped_cloud_pub4.publish(*merged4);
	else
		cropped_cloud_pub4.publish(emptyCloud2);
	if (merged3hit)
		cropped_cloud_pub3.publish(*merged3);
	else
		cropped_cloud_pub3.publish(emptyCloud2);
	if (merged2hit)
		cropped_cloud_pub2.publish(*merged2);
	else
		cropped_cloud_pub2.publish(emptyCloud2);
	if (merged1hit)
		cropped_cloud_pub1.publish(*merged1);
	else
		cropped_cloud_pub1.publish(emptyCloud2);


	double frontpred = predictionMatrix[front_index][0];
	ROS_INFO("frontpred: %f", frontpred);
	ROS_INFO("prediction_memory: %f", prediction_memory);
	prediction_memory = (1 - memory_decay_rate) * prediction_memory
	  + memory_decay_rate * frontpred;
	ROS_INFO("updated prediction_memory: %f", prediction_memory);

	firstCloudProcessed = true;

	delete merged1;
	delete merged2;
	delete merged3;
	delete merged4;
	delete[] pcdArray;
	shouldProcess = true;
	// ROS_INFO("Done processing cloud -------------------------------");
} // process_cloud

void Driver::nonlinear_steering()
{
	double controller_speed;
	double controller_omega;

	double tx = cos(des_state_psi); // [tx,ty] is tangent of desired path
	double ty = sin(des_state_psi);
	double nx = -ty; // components [nx, ny] of normal to path, points to left of desired heading
	double ny = tx;

	gms_client.call(getmodelstate);
	double state_x = getmodelstate.response.pose.position.x;
	double state_y = getmodelstate.response.pose.position.y;

	double state_psi = getRobotYaw();

	double dx = state_x - des_state_x; // x-error relative to desired path
	double dy = state_y - des_state_y; // y-error

	double lateral_err = dx * nx + dy * ny; // lateral error is error vector dotted with path normal
	// lateral offset error is positive if robot is to the left of the path
	double trip_dist_err = dx * tx + dy * ty; // progress error: if +, then we are ahead of schedule
	// heading error: if positive, should rotate -omega to align with desired heading
	double heading_err  = min_dang(state_psi - des_state_psi);
	double strategy_psi = psi_strategy(lateral_err); // heading command, based on NL algorithm
	controller_omega = omega_cmd_fnc(strategy_psi, state_psi, des_state_psi);

	// set the desired speed, changes based on how fast the robot is going relative to the desired speed
	double current_speed = abs(getmodelstate.response.twist.linear.x)
	  + abs(getmodelstate.response.twist.linear.y)
	  + abs(getmodelstate.response.twist.linear.z);

	double diff = current_speed - TARGET_SPEED;
	if (diff < 0) {
		double fraction = abs(diff / TARGET_SPEED);
		if (fraction > 1) {
			fraction = 1;
		}
		controller_speed = TARGET_SPEED + MAX_BOOST * fraction;
		// ROS_WARN("controller_speed1 is: %f", controller_speed);
	} else {
		double fraction = abs(diff / TARGET_SPEED);
		if (fraction > 1) {
			fraction = 1;
		}
		// diff is positive so send cmd to slow down (up to 3/4 of target speed)
		controller_speed = 3 * TARGET_SPEED / 4 + TARGET_SPEED / 4 * (1 - fraction);
		// ROS_WARN("controller_speed2 is: %f", controller_speed);
	}

	geometry_msgs::Twist twist_cmd;
	// send out our speed/spin commands:
	twist_cmd.linear.x  = controller_speed;
	twist_cmd.angular.z = controller_omega;

	// TODO uncomment to send drive commands
	cmd_publisher.publish(twist_cmd);
} // nonlinear_steering

// utility fnc to compute min delta angle, accounting for periodicity
double Driver::min_dang(double dang)
{
	while (dang > M_PI) dang -= 2.0 * M_PI;
	while (dang < -M_PI) dang += 2.0 * M_PI;
	return dang;
}

// saturation function, values -1 to 1
double Driver::sat(double x)
{
	if (x > 1.0) {
		return 1.0;
	}
	if (x < -1.0) {
		return -1.0;
	}
	return x;
}

double Driver::psi_strategy(double offset_err)
{
	double psi_strategy = -(M_PI / 2) * sat(offset_err / K_LAT_ERR_THRESH);

	return psi_strategy;
}

double Driver::omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path)
{
	double psi_cmd   = psi_strategy + psi_path;
	double omega_cmd = K_PSI * (psi_cmd - psi_state);

	omega_cmd = MAX_OMEGA * sat(omega_cmd / MAX_OMEGA); // saturate the command at specified limit
	return omega_cmd;
}

void Driver::stop()
{
	// ROS_WARN("Stopping...");
	geometry_msgs::Twist twist_cmd;
	double current_speed = 100000; // init at some high number

	while (current_speed > 0.05) {
		gms_client.call(getmodelstate);
		current_speed = abs(getmodelstate.response.twist.linear.x)
		  + abs(getmodelstate.response.twist.linear.y)
		  + abs(getmodelstate.response.twist.linear.z);
		twist_cmd.linear.x  = 0;
		twist_cmd.angular.z = 0;
		cmd_publisher.publish(twist_cmd);
	}
	// ROS_WARN("Stopped");
} // stop

void Driver::sendRotateCmd(double direction)
{
	gms_client.call(getmodelstate);
	double current_angular = abs(getmodelstate.response.twist.angular.x)
	  + abs(getmodelstate.response.twist.angular.y)
	  + abs(getmodelstate.response.twist.angular.z);

	double omega_cmd;
	double diff = current_angular - TARGET_OMEGA;
	if (diff < 0) {
		double fraction = abs(diff / TARGET_OMEGA);
		if (fraction > 1) {
			fraction = 1;
		}
		omega_cmd = TARGET_OMEGA + MAX_OMEGA_BOOST * fraction;
	} else {
		double fraction = abs(diff / TARGET_OMEGA);
		if (fraction > 1) {
			fraction = 1;
		}
		// diff is positive so send cmd to slow down (up to 3/4 of target speed)
		omega_cmd = 3 * TARGET_OMEGA / 4 + TARGET_OMEGA / 4 * (1 - fraction);
	}
	omega_cmd = omega_cmd * direction;
	// ROS_INFO("omega_cmd: %f", omega_cmd);
	geometry_msgs::Twist twist_cmd;
	twist_cmd.linear.x  = 0;
	twist_cmd.angular.z = omega_cmd;

	// TODO uncomment to send drive commands
	cmd_publisher.publish(twist_cmd);
}

double Driver::getRobotYaw()
{
	gms_client.call(getmodelstate);
	tf::Quaternion q(getmodelstate.response.pose.orientation.x,
	  getmodelstate.response.pose.orientation.y,
	  getmodelstate.response.pose.orientation.z,
	  getmodelstate.response.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void Driver::voxelFilter(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& voxel_out)
{
	pcl::PCLPointCloud2 pcl_pc;

	pcl_conversions::toPCL(input, pcl_pc);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);
	// Set up VoxelGrid filter to bin into 5cm grid
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(input_ptr);
	sor.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	sor.filter(*output_ptr);
	pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
	pcl_conversions::fromPCL(pcl_pc, voxel_out);
}

void Driver::transformYAxis(double yaw, pcl::PointCloud<pcl::PointXYZ>& cloud,
  pcl::PointCloud<pcl::PointXYZ>& transformed_cloud, double xtrans, double ztrans)
{
	Eigen::Matrix<double, 4, 4> tmp;
	// rotation about the y axis (relative to the kinect frame).
	// Equivalent to rotating about  the z axis in gazebo frame, which is
	// why we're inputing the yaw rotation we want
	tmp(0, 0) = cos(yaw);
	tmp(0, 1) = 0;
	tmp(0, 2) = sin(yaw);
	tmp(0, 3) = xtrans; // x axis translation
	tmp(1, 0) = 0;
	tmp(1, 1) = 1;
	tmp(1, 2) = 0;
	tmp(1, 3) = 0; // y axis translation
	tmp(2, 0) = -sin(yaw);
	tmp(2, 1) = 0;
	tmp(2, 2) = cos(yaw);
	tmp(2, 3) = ztrans; // z axis translation
	tmp(3, 0) = 0;
	tmp(3, 1) = 0;
	tmp(3, 2) = 0;
	tmp(3, 3) = 1;
	const Eigen::Matrix<double, 4, 4> ytransform(tmp);
	pcl::transformPointCloud(cloud, transformed_cloud, ytransform);
}

void Driver::pointCloudCallback(const sensor_msgs::PointCloud2& input)
{
	if (!merging_cloud) {
		sensor_msgs::PointCloud2 voxel_out;
		voxelFilter(input, voxel_out);
		// no need to transform, this is the default kinect
		kinectcloud = voxel_out;
	}
	if (shouldProcess && !merging_cloud && firstCloudSeen) {
		merging_cloud = true;
		sensor_msgs::PointCloud2 * merged = new sensor_msgs::PointCloud2();
		pcl::concatenatePointCloud(kinectcloud, kinectcloud1, *merged);
		sensor_msgs::PointCloud2 * merged1 = new sensor_msgs::PointCloud2();
		pcl::concatenatePointCloud(*merged, kinectcloud2, *merged1);
		sensor_msgs::PointCloud2 * merged2 = new sensor_msgs::PointCloud2();
		pcl::concatenatePointCloud(*merged1, kinectcloud3, *merged2);
		process_cloud(*merged2);
		merging_cloud = false;
		delete merged;
		delete merged1;
		delete merged2;
	}
	firstCloudSeen = true;
}

void Driver::pointCloudCallback1(const sensor_msgs::PointCloud2& input)
{
	// first apply voxel filter to reduce number of points
	sensor_msgs::PointCloud2 voxel_out;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	voxelFilter(input, voxel_out);
	pcl::fromROSMsg(voxel_out, cloud);
	double yaw    = -M_PI / 2;
	double xtrans = -bodyLen / 2;
	double ztrans = -bodyLen / 2;
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	transformYAxis(yaw, cloud, transformed_cloud, xtrans, ztrans);
	if (!merging_cloud) {
		pcl::toROSMsg(transformed_cloud, kinectcloud1);
	}
} // pointCloudCallback1

void Driver::pointCloudCallback2(const sensor_msgs::PointCloud2& input)
{
	sensor_msgs::PointCloud2 voxel_out;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	voxelFilter(input, voxel_out);
	pcl::fromROSMsg(voxel_out, cloud);
	double yaw    = M_PI;
	double xtrans = 0;
	double ztrans = -bodyLen;
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	transformYAxis(yaw, cloud, transformed_cloud, xtrans, ztrans);
	if (!merging_cloud) {
		pcl::toROSMsg(transformed_cloud, kinectcloud2);
	}
}

void Driver::pointCloudCallback3(const sensor_msgs::PointCloud2& input)
{
	sensor_msgs::PointCloud2 voxel_out;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	voxelFilter(input, voxel_out);
	pcl::fromROSMsg(voxel_out, cloud);
	double yaw    = M_PI / 2;
	double xtrans = bodyLen / 2;
	double ztrans = -bodyLen / 2;
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	transformYAxis(yaw, cloud, transformed_cloud, xtrans, ztrans);
	if (!merging_cloud) {
		pcl::toROSMsg(transformed_cloud, kinectcloud3);
	}
}

void Driver::imuCallback(const sensor_msgs::Imu& imu_in)
{
	imu_orientation.x     = imu_in.orientation.x;
	imu_orientation.y     = imu_in.orientation.y;
	imu_orientation.z     = imu_in.orientation.z;
	imu_orientation.w     = imu_in.orientation.w;
	linear_acceleration.x = imu_in.linear_acceleration.x;
	linear_acceleration.y = imu_in.linear_acceleration.y;
	linear_acceleration.z = imu_in.linear_acceleration.z;
	double total_accel = abs(linear_acceleration.x) + abs(linear_acceleration.y)
	  + abs(linear_acceleration.z);
	imuReady = true;
}

int Driver::Go()
{
	ROS_WARN("Waiting for first cloud to be processed...");
	while (!firstCloudProcessed) {
		ros::spinOnce();
		half_sec.sleep();
	}
	ROS_WARN("starting control loop");

	double carrot_dist = 10.0;

	while (true) {
		// ROS_INFO("Starting stroll");
		ros::spinOnce();
		des_state_psi = getRobotYaw();

		double last_x = getmodelstate.response.pose.position.x;
		double last_y = getmodelstate.response.pose.position.y;
		// generate a 'carrot' for the robot, a point in front of it that it
		// will never actually reach because it keeps moving
		des_state_x = cos(des_state_psi) * carrot_dist + last_x;
		des_state_y = sin(des_state_psi) * carrot_dist + last_y;

		ROS_INFO("prediction_memory: %f", prediction_memory);
		if (prediction_memory <= 0.5) {
			ROS_WARN("DANGER, WILL ROBINSON, DANGER!!!");

			stop(); // returns when stopped

			// double random = ((double) rand() / (RAND_MAX));
			// if (random < 0.5) {
			//  random = -1;
			// } else {
			//  random = 1;
			// }

			double yaw;
			double desired_yaw;
			double direction;
			bool foundpath = false;
			for (int i = 1; i <= NUM_AREAS / 4; i++) {
				yaw = getRobotYaw();
				int index   = front_index + i;
				double pred = predictionMatrix[index][0];
				if (pred > 0.5) {
					foundpath   = true;
					direction   = 1;
					desired_yaw = yaw + direction * i * DELTA_PSI;
					break;
				}

				index = front_index - i;
				if (index < 0)
					index += 12;
				pred = predictionMatrix[index][0];

				if (pred > 0.5) {
					foundpath   = true;
					direction   = -1;
					desired_yaw = yaw + direction * i * DELTA_PSI;
					break;
				}
			}
			if (!foundpath) {
				ROS_WARN("uh oh, clear patch not found... yolo rotating anyway");
				direction   = 1;
				desired_yaw = yaw + direction * DELTA_PSI;
			}

			if (desired_yaw > M_PI) {
				// because angle must be in range from -PI to PI
				desired_yaw = desired_yaw - 2 * M_PI;
			} else if (desired_yaw < -1 * M_PI) {
				desired_yaw = desired_yaw + 2 * M_PI;
			}
			// ROS_WARN("direction is: %f", direction);
			// ROS_WARN("yaw is: %f", yaw);
			// ROS_WARN("desired_yaw is: %f", desired_yaw);


			// double desired_yaw2 = desired_yaw + M_PI; // add 180 degrees

			double diff = desired_yaw - yaw;
			while ((direction == 1 && ((M_PI > diff && diff > 0) || diff < -1 * M_PI)) ||
			  (direction == -1 && !((M_PI > diff && diff > 0) || diff < -1 * M_PI)))
			{
				yaw = getRobotYaw();
				sendRotateCmd(direction);
				diff = desired_yaw - yaw;
				// ROS_INFO("yaw2: %f", yaw);
				// ROS_INFO("desired_yaw2: %f", desired_yaw);
				// ROS_WARN("diff: %f", diff);
			}

			prediction_memory = 0.5; // reset to unbiased value because we finished turning

			// half_sec.sleep();
			// half_sec.sleep();
			// half_sec.sleep();
			des_state_psi = getRobotYaw();
			last_x        = getmodelstate.response.pose.position.x;
			last_y        = getmodelstate.response.pose.position.y;
			// generate a 'carrot' for the robot, a point in front of it that it
			// will never actually reach because it keeps moving
			des_state_x = cos(des_state_psi) * carrot_dist + last_x;
			des_state_y = sin(des_state_psi) * carrot_dist + last_y;
		} else {
			ROS_INFO("good to go");
		}
		nonlinear_steering();
		ROS_INFO("-------------------------------------");
	}
	return 0;
} // Go

int main(int argc, char ** argv)
{
	ROS_INFO("Starting driver");
	ros::init(argc, argv, "driver", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	Driver driver(&n);
	driver.Go();

	// should never get here
	return 0;
}
