#include "data_collector.h"

// constructor
DataCollector::DataCollector(ros::NodeHandle * nodehandle) : n(*nodehandle)
{
	shouldCapture  = false;
	imuReady       = false;
	contactFailure = false;
	fileCounter    = 0;

	des_state_speed = TARGET_SPEED;
	des_state_omega = 0.0;
	// hard code a simple path: the world x axis
	des_state_x   = 0.0;
	des_state_y   = 0.0;
	des_state_psi = 0.0; // M_PI/2;
	half_sec      = ros::Duration(0.5);
	five_ms       = ros::Duration(0.005);

	imu_subscriber = n.subscribe("/imu_quat", 1, &DataCollector::imuCallback, this);

	ros::spinOnce();
	while (!imuReady) {
		ROS_INFO("waiting for imu");
		ros::spinOnce();
		half_sec.sleep();
	}

	cropped_cloud_pub  = n.advertise<sensor_msgs::PointCloud2>("/cropped_cloud", 1);
	cmd_publisher      = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	pcl_subscriber     = n.subscribe("/kinect/depth/points", 1, &DataCollector::pointCloudCallback, this);
	contact_subscriber = n.subscribe("/lidar_bumper_state", 1, &DataCollector::lidarBumperCallback, this);

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
}

// save as RAW
void DataCollector::saveImage(int ** imageMatrix,
  int xdim, int ydim, char valuesFile[], char imageFile[])
{
	// ROS_INFO("Writing Labeled Image to Disk");
	// int numys = sizeof( imageMatrix[0] ) / sizeof( imageMatrix[0][0] );
	// int numxs = sizeof( imageMatrix ) / sizeof( imageMatrix[0] );
	// ROS_INFO("xdim,ydim,numys,numxs %d %d %d %d", xdim, ydim, numys, numxs);

	char ppath[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_OUTPUT")];
	char ppathfull[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_OUTPUT_FULL")];
	char npath[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_OUTPUT")];
	char npathfull[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_OUTPUT_FULL")];
	char txtppath[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_txtOUTPUT")];
	char txtppathfull[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_txtOUTPUT_FULL")];
	char txtnpath[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_txtOUTPUT")];
	char txtnpathfull[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_txtOUTPUT_FULL")];

	strcpy(ppath, PATH_PREFIX);
	strcpy(ppathfull, PATH_PREFIX);
	strcpy(npath, PATH_PREFIX);
	strcpy(npathfull, PATH_PREFIX);
	strcpy(txtppath, PATH_PREFIX);
	strcpy(txtppathfull, PATH_PREFIX);
	strcpy(txtnpath, PATH_PREFIX);
	strcpy(txtnpathfull, PATH_PREFIX);
	ppath[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_OUTPUT")];
	ppathfull[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_OUTPUT_FULL")];
	npath[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_OUTPUT")];
	npathfull[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_OUTPUT_FULL")];
	txtppath[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_txtOUTPUT")];
	txtppathfull[strlen(PATH_PREFIX) + strlen("POSITIVE_CROPPED_txtOUTPUT_FULL")];
	txtnpath[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_txtOUTPUT")];
	txtnpathfull[strlen(PATH_PREFIX) + strlen("NEGATIVE_CROPPED_txtOUTPUT_FULL")];
	strncat(ppath, "POSITIVE_CROPPED_OUTPUT", strlen("POSITIVE_CROPPED_OUTPUT"));
	strncat(ppathfull, "POSITIVE_CROPPED_OUTPUT_FULL", strlen("POSITIVE_CROPPED_OUTPUT_FULL"));
	strncat(npath, "NEGATIVE_CROPPED_OUTPUT", strlen("NEGATIVE_CROPPED_OUTPUT"));
	strncat(npathfull, "NEGATIVE_CROPPED_OUTPUT_FULL", strlen("NEGATIVE_CROPPED_OUTPUT_FULL"));
	strncat(txtppath, "POSITIVE_CROPPED_txtOUTPUT", strlen("POSITIVE_CROPPED_txtOUTPUT"));
	strncat(txtppathfull, "POSITIVE_CROPPED_txtOUTPUT_FULL", strlen("POSITIVE_CROPPED_txtOUTPUT_FULL"));
	strncat(txtnpath, "NEGATIVE_CROPPED_txtOUTPUT", strlen("NEGATIVE_CROPPED_txtOUTPUT"));
	strncat(txtnpathfull, "NEGATIVE_CROPPED_txtOUTPUT_FULL", strlen("NEGATIVE_CROPPED_txtOUTPUT_FULL"));
	// ROS_WARN("-------------------1");
	boost::filesystem::create_directories(ppath);
	// ROS_WARN("-------------------2");
	boost::filesystem::create_directories(ppathfull);
	// ROS_WARN("-------------------3");
	boost::filesystem::create_directories(npath);
	// ROS_WARN("-------------------4");
	boost::filesystem::create_directories(npathfull);
	// ROS_WARN("-------------------5");
	boost::filesystem::create_directories(txtppath);
	// ROS_WARN("-------------------6");
	boost::filesystem::create_directories(txtppathfull);
	// ROS_WARN("-------------------7");
	boost::filesystem::create_directories(txtnpath);
	// ROS_WARN("-------------------8");
	boost::filesystem::create_directories(txtnpathfull);
	// ROS_WARN("-------------------9");

	ofstream heightfile(valuesFile, ios::out | ios::binary);
	FILE * image = fopen(imageFile, "wb");

	for (unsigned int yIndex = 0; yIndex < ydim; yIndex++) {
		for (unsigned int xIndex = 0; xIndex < xdim; xIndex++) {
			// write the values to a separate file so we can see the numbers
			if (xIndex == 0) {
				heightfile << "\n";
			}
			heightfile << imageMatrix[xIndex][yIndex] << ", ";
			// now write the actual image
			fwrite(&imageMatrix[xIndex][yIndex], sizeof(unsigned char), 1, image);
		}
	}
	heightfile.close();
	fclose(image);
	// ROS_INFO("done writing image");
} // saveImage

void DataCollector::take_snapshot(const sensor_msgs::PointCloud2& input)
{
	// get the most updated location
	gms_client.call(getmodelstate);
	shouldCapture = false;
	// ROS_INFO("Taking snapshot");
	pcl::PointCloud<pcl::PointXYZ> cloud;
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
	tmp(1, 3) = 0; // y axis translation
	tmp(2, 0) = 0;
	tmp(2, 1) = sin(theta);
	tmp(2, 2) = cos(theta);
	tmp(2, 3) = 0; // z axis translation
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
	tmp(1, 3) = 0; // y axis translation
	tmp(2, 0) = 0;
	tmp(2, 1) = 0;
	tmp(2, 2) = 1;
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

	// for debug purposes
	double maxX = -100;
	double maxY = -100;
	double maxZ = -100;
	double minX = 100;
	double minY = 100;
	double minZ = 100;

	// we're interesting only in the 1m x 1m square in front of the robot
	// double xMin = -0.5;
	// double xMax = 0.5;
	// double yMin = 0.5;
	// double yMax = 1.5;
	double xMin = -targetWidth / 2;
	double xMax = targetWidth / 2;
	double yMin = frontSpacing;
	double yMax = yMin + stripLen;

	// and we need to limit the height range
	// height range should be be centered on ground instead of the height of the wobbler,
	// wobbler is about 1 meter of the ground (but wobbler height is considered height 0)
	double zMin = -5;
	double zMax = 3;

	int xdim = (int) (targetWidth / resolution);
	int ydim = (int) (stripLen / resolution);
	double intensities[xdim * ydim];

	// initialize with nan
	for (int i = 0; i < xdim * ydim; i++) {
		intensities[i] = NAN;
	}

	for (int i = 0; i < cloudsize; i++) {
		// in the gazebo's frame the point cloud's y is in the -z direction,
		// z is in the y direction, and x is the same
		double x = cloud[i].x;
		double y = cloud[i].z;
		double z = -cloud[i].y;

		// only care about points that fall in our square
		if (xMin < x && xMax > x &&
		  yMin < y && yMax > y)
		{
			// for debug purposes
			if (x < minX) {
				minX = x;
			}
			if (y < minY) {
				minY = y;
			}
			if (z < minZ) {
				minZ = z;
			}
			if (x > maxX) {
				maxX = x;
			}
			if (y > maxY) {
				maxY = y;
			}
			if (z > maxZ) {
				maxZ = z;
			}
			double fraction = (x - xMin) / (xMax - xMin);
			// round by adding 0.5 and casting (truncating)
			int xIndex = (int) ((xdim - 1) * fraction + 0.5);
			fraction = (y - yMin) / (yMax - yMin);
			int yIndex = (int) ((ydim - 1) * fraction + 0.5);
			int index  = xIndex + yIndex * xdim;
			fraction = (z - zMin) / (zMax - zMin);
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
			if (isnan(intensities[index]) || intensities[index] < value) {
				intensities[index] = value;
			}
		} else {
			// otherwise we don't want these points, set them to NAN
			cloud[i].x = NAN;
			cloud[i].z = NAN;
			cloud[i].y = NAN;
		}
	}
	// debug purposes, so we can see the range of our square in all 3 dimensions
	//	ofstream bounds("bounds.txt", ios::out | ios::binary);
	//	bounds << minX << "\n";
	//	bounds << maxX << "\n";
	//	bounds << minY << "\n";
	//	bounds << maxY << "\n";
	//	bounds << minZ << "\n";
	//	bounds << maxZ << "\n";
	//	bounds.close();

	// convert intensities array of doubles to ints for the imageMatrix
	int ** imageMatrix;
	imageMatrix = new int *[xdim];

	for (unsigned int xIndex = 0; xIndex < xdim; xIndex++) {
		imageMatrix[xIndex] = new int[ydim];
		for (unsigned int yIndex = 0; yIndex < ydim; yIndex++) {
			int index = xIndex + yIndex * xdim;
			if (isnan(intensities[index])) {
				// anything still nan is unknown value so set it to 255 to
				// separate it from the 0-200 range
				imageMatrix[xIndex][yIndex] = 255;
			} else {
				// add 0.5 to round instead of truncating
				imageMatrix[xIndex][yIndex] = (int) (intensities[index] + 0.5);
			}
		}
	}

	// add imagematrix/position struct to the list of structs
	struct ImageStruct imageStruct;
	imageStruct.imageMatrix = imageMatrix;
	imageStruct.x = getmodelstate.response.pose.position.x;
	imageStruct.y = getmodelstate.response.pose.position.y;
	imagesStructVector.push_back(imageStruct);

	// ROS_WARN("Just saved imgstruct with x,y = %f %f", imageStruct.x, imageStruct.y);

	// ostringstream ss;
	// ss << fileCounter;
	// string str = ss.str();
	// char suffix[str.length()];
	// strcpy(suffix, str.c_str());
	// char valuesFile[strlen(suffix) + strlen("./****************_OUTPUT/fullvalues.txt")];
	// char imageFile[strlen(suffix) + strlen("./****************_OUTPUT/fullheightmap.data")];
	// strcpy(valuesFile, "./POSITIVE_CROPPED_OUTPUT/fullvalues");
	// strcpy(imageFile, "./POSITIVE_CROPPED_OUTPUT/fullheightmap");
	// strncat(valuesFile, suffix, strlen(suffix));
	// strncat(valuesFile, ".txt", strlen(".txt"));
	// strncat(imageFile, suffix, strlen(suffix));
	// strncat(imageFile, ".data", strlen(".data"));
	// saveImage(imageMatrix, xdim, ydim, valuesFile, imageFile);
	// fileCounter++;

	sensor_msgs::PointCloud2 croppedCloud;
	pcl::toROSMsg(cloud, croppedCloud);

	if (!cropped_cloud_pub) {
		ROS_WARN("Publisher invalid!");
	} else {
		cropped_cloud_pub.publish(croppedCloud);
	}
} // take_snapshot

void DataCollector::saveCroppedImages(double xcoord, double ycoord, bool label)
{
	ROS_INFO("-------------------------------------------------");
	int num_candidates = imagesStructVector.size();
	ROS_INFO("Saving batch of %d candidates images", num_candidates);

	for (int i = 0; i < num_candidates; i++) {
		int ** imgMatrix = imagesStructVector[i].imageMatrix;
		double x         = imagesStructVector[i].x;
		double y         = imagesStructVector[i].y;

		// ROS_WARN("Just Read out an imgstruct with x,y = %f %f", x, y);

		double dist_from_capture = sqrt(pow(xcoord - x, 2) + pow(ycoord - y, 2));
		double avg_heading       = atan2(ycoord - y, xcoord - x);
		double angle_offset      = des_state_psi - avg_heading;
		double dist_along_path   = cos(angle_offset) * dist_from_capture;

		int yIndexStart = (int) ((dist_along_path - frontSpacing - bodyLen) / resolution);
		int xdim        = (int) (targetWidth / resolution);
		int ydim        = (int) (targetLen / resolution);
		int yIndexEnd   = yIndexStart + ydim - 1;

		// ROS_INFO("dist %f, des_state_psi %f, avg_head %f, angle_off %f, dist_along_path %f,"
		//   "yIndexStart %d, yIndexEnd %d: ", dist_from_capture, des_state_psi, avg_heading,
		//   angle_offset, dist_along_path, yIndexStart, yIndexEnd);
		// ROS_INFO("xdim,ydim: %d %d", xdim, ydim);

		int ** croppedImageMatrix = new int *[xdim];
		int num_pixels_y = (int) (stripLen / resolution);
		// skip this image if the area of interest is not fully contained in the strip
		if (yIndexStart >= 0 && yIndexEnd < num_pixels_y) {
			for (unsigned int xIndex = 0; xIndex < xdim; xIndex++) {
				croppedImageMatrix[xIndex] = new int[ydim];
				for (unsigned int yIndex = 0; yIndex < ydim; yIndex++) {
					// The image starts counting pixels from the side furthest from the robot
					croppedImageMatrix[xIndex][yIndex] =
					  imgMatrix[xIndex][yIndex + yIndexStart];
					// imgMatrix[xIndex][num_pixels_y - 1 - (yIndex + yIndexStart)];
				}
			}

			ROS_INFO("saving a valid cropping, yIndexStart, yIndexEnd: %d %d", yIndexStart, yIndexEnd);

			ostringstream ss;
			ss << fileCounter;
			string str = ss.str();
			char suffix[str.length()];
			strcpy(suffix, str.c_str());
			char valuesFile[strlen(suffix) + strlen(PATH_PREFIX)   \
			  + strlen("****************_txtOUTPUT/values.txt")];
			char imageFile[strlen(suffix) + strlen(PATH_PREFIX)	  \
			  + strlen("****************_OUTPUT/heightmap.data")];
			strcpy(valuesFile, PATH_PREFIX);
			strcpy(imageFile, PATH_PREFIX);
			if (label) {
				strncat(valuesFile, "POSITIVE_CROPPED_txtOUTPUT/values",
				  strlen("****************_txtOUTPUT/values.txt"));
				strncat(imageFile, "POSITIVE_CROPPED_OUTPUT/heightmap",
				  strlen("****************_OUTPUT/heightmap.data"));
			} else {
				strncat(valuesFile, "NEGATIVE_CROPPED_txtOUTPUT/values",
				  strlen("****************_txtOUTPUT/values.txt"));
				strncat(imageFile, "NEGATIVE_CROPPED_OUTPUT/heightmap",
				  strlen("****************_OUTPUT/heightmap.data"));
			}
			strncat(valuesFile, suffix, strlen(suffix));
			strncat(valuesFile, ".txt", strlen(".txt"));
			strncat(imageFile, suffix, strlen(suffix));
			strncat(imageFile, ".data", strlen(".data"));

			// write to disk
			saveImage(croppedImageMatrix, xdim, ydim, valuesFile, imageFile);


			// TODO comment out
			int xdim2 = (int) (targetWidth / resolution);
			int ydim2 = (int) (stripLen / resolution);
			char valuesFile2[strlen(suffix) + strlen(PATH_PREFIX)	\
			  + strlen("****************_txtOUTPUT_FULL/values.txt")];
			char imageFile2[strlen(suffix) + strlen(PATH_PREFIX)	  \
			  + strlen("****************_OUTPUT_FULL/heightmap.data")];
			strcpy(valuesFile2, PATH_PREFIX);
			strcpy(imageFile2, PATH_PREFIX);
			if (label) {
				strncat(valuesFile2, "POSITIVE_CROPPED_txtOUTPUT_FULL/values",
				  strlen("****************_txtOUTPUT_FULL/values.txt"));
				strncat(imageFile2, "POSITIVE_CROPPED_OUTPUT_FULL/heightmap",
				  strlen("****************_OUTPUT_FULL/heightmap.data"));
			} else {
				strncat(valuesFile2, "NEGATIVE_CROPPED_txtOUTPUT_FULL/values",
				  strlen("****************_txtOUTPUT_FULL/values.txt"));
				strncat(imageFile2, "NEGATIVE_CROPPED_OUTPUT_FULL/heightmap",
				  strlen("****************_OUTPUT_FULL/heightmap.data"));
			}
			strncat(valuesFile2, suffix, strlen(suffix));
			strncat(valuesFile2, ".txt", strlen(".txt"));
			strncat(imageFile2, suffix, strlen(suffix));
			strncat(imageFile2, ".data", strlen(".data"));
			saveImage(imgMatrix, xdim2, ydim2, valuesFile2, imageFile2);
			// TODO comment out

			fileCounter++;
		} else {
			ROS_INFO("skipped out of range cropping, yIndexStart, yIndexEnd: %d %d", yIndexStart, yIndexEnd);
		}
	}
	ROS_INFO("***************************************************");
	// empty imagesStructVector so it can be filled again later with new data
} // saveCroppedImages

void DataCollector::nonlinear_steering()
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

	tf::Quaternion q(getmodelstate.response.pose.orientation.x,
	  getmodelstate.response.pose.orientation.y,
	  getmodelstate.response.pose.orientation.z,
	  getmodelstate.response.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	double state_psi = yaw;

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
	} else {
		double fraction = abs(diff / MAX_BOOST);
		if (fraction > 1) {
			fraction = 1;
		}
		controller_speed = TARGET_SPEED * (1 - fraction);
	}

	geometry_msgs::Twist twist_cmd;
	// send out our speed/spin commands:
	twist_cmd.linear.x  = controller_speed;
	twist_cmd.angular.z = controller_omega;
	cmd_publisher.publish(twist_cmd);

	// DEBUG OUTPUT...
	//	ROS_INFO("des_state_psi, heading err = %f, %f", des_state_psi,heading_err);
	//	ROS_INFO("lateral err, trip dist err = %f, %f",lateral_err,trip_dist_err);

	//	std_msgs::Float32 float_msg;
	//	float_msg.data = lateral_err_;
	//	lat_err_publisher.publish(float_msg);
	//	float_msg.data = state_psi_;
	//	heading_publisher.publish(float_msg);
	//	float_msg.data = psi_cmd_;
	//	heading_cmd_publisher.publish(float_msg);
	// END OF DEBUG OUTPUT
} // nonlinear_steering

// utility fnc to compute min delta angle, accounting for periodicity
double DataCollector::min_dang(double dang)
{
	while (dang > M_PI) dang -= 2.0 * M_PI;
	while (dang < -M_PI) dang += 2.0 * M_PI;
	return dang;
}

// saturation function, values -1 to 1
double DataCollector::sat(double x)
{
	if (x > 1.0) {
		return 1.0;
	}
	if (x < -1.0) {
		return -1.0;
	}
	return x;
}

double DataCollector::psi_strategy(double offset_err)
{
	double psi_strategy = -(M_PI / 2) * sat(offset_err / K_LAT_ERR_THRESH);

	return psi_strategy;
}

double DataCollector::omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path)
{
	double psi_cmd   = psi_strategy + psi_path;
	double omega_cmd = K_PSI * (psi_cmd - psi_state);

	omega_cmd = MAX_OMEGA * sat(omega_cmd / MAX_OMEGA); // saturate the command at specified limit
	return omega_cmd;
}

// check if the robot is upright and able to drive
bool DataCollector::checkIfUpwright(gazebo_msgs::GetModelState gms)
{
	// robot is unrealistically bottom heavy to discourage tipping (makes spawning easier)
	// so we set conservative tilt to cause failure.
	// Tolerance for upright is a roll of < 40 degrees and a pitch of < 55 degrees (0.698 and 0.96 rad)
	tf::Quaternion q(gms.response.pose.orientation.x,
	  gms.response.pose.orientation.y,
	  gms.response.pose.orientation.z,
	  gms.response.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;

	m.getRPY(roll, pitch, yaw);
	if (abs(pitch) < 0.96 && abs(roll) < 0.698) {
		//		ROS_INFO("UPRIGHT, rpy is: %f, %f, %f", roll, pitch, yaw);
		return true;
	} else {
		//		ROS_INFO("NOT UPRIGHT, rpy is: %f, %f, %f", roll, pitch, yaw);
		return false;
	}
}

void DataCollector::sendStopCmd()
{
	geometry_msgs::Twist twist_cmd;

	// stop the robit
	twist_cmd.linear.x  = 0;
	twist_cmd.angular.z = 0;
	cmd_publisher.publish(twist_cmd);
}

bool DataCollector::checkForPhysicalFailures(gazebo_msgs::GetModelState gms, bool shouldSaveImages)
{
	double x = getmodelstate.response.pose.position.x;
	double y = getmodelstate.response.pose.position.y;

	// check if the robot has bumped into anything
	if (contactFailure) {
		ROS_WARN("FAILURE DETECTED!!! - contact failure");
		sendStopCmd();
		if (shouldSaveImages) {
			saveCroppedImages(x, y, false); // save with negative label
		}
		return true;
	}

	// check if the robot is out of control (ie going too fast down a hill or falling)
	double total_linear = abs(gms.response.twist.linear.x)
	  + abs(gms.response.twist.linear.y)
	  + abs(gms.response.twist.linear.z);
	if (total_linear > FAILURE_SPEED) {
		ROS_WARN("FAILURE DETECTED!!! - exceeded failure speed");
		sendStopCmd();
		return true;
	}

	// see if the robot has falled over
	if (!checkIfUpwright(gms)) {
		ROS_WARN("FAILURE DETECTED!!! - tipped too far");
		sendStopCmd();
		// now create and save the negatively labeled images
		if (shouldSaveImages) {
			saveCroppedImages(x, y, false); // save with negative label
		}
		return true;
	}

	return false;
} // checkForPhysicalFailures

void DataCollector::pointCloudCallback(const sensor_msgs::PointCloud2& cloud)
{
	if (shouldCapture) {
		take_snapshot(cloud);
	}
}

void DataCollector::lidarBumperCallback(const gazebo_msgs::ContactsState& stateList)
{
	// any items in the list include the lidar as one of the colliding objects
	if (!stateList.states.empty()) {
		// ROS_INFO("Lidar collision detected!!");
		contactFailure = true;
	}
}

void DataCollector::imuCallback(const sensor_msgs::Imu& imu_in)
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

int DataCollector::mainLoop()
{
	ROS_INFO("Starting data collection loop");

	while (true) {
		ros::spinOnce();
		outputPositive = false;
		// when spawning the robot, we drop it from a height higher than the highest point on the map,
		// wait till it hits (can till from imu), quickly get the x,y,z from the gazevbo model state,
		// and respawn slightly above that x,y,z point.
		gazebo_msgs::SetModelState model_state_srv_msg;
		model_state_srv_msg.request.model_state.model_name = "robot_description";

		double xDisplacement;
		double yDisplacement;
		// get 2 random numbers in range -1 to 1
		double xmultiplier = ((double) rand() / (RAND_MAX)) * 2.0 - 1.0;
		double ymultiplier = ((double) rand() / (RAND_MAX)) * 2.0 - 1.0;
		double spawnx      = xmultiplier * 12; // 47 for the larger map, 12 for smaller map
		double spawny      = ymultiplier * 12; // 47 for the larger map, 12 for smaller map
		double angle       = ((double) rand() / (RAND_MAX)) * 2 * M_PI;
		model_state_srv_msg.request.model_state.pose.orientation =
		  tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
		model_state_srv_msg.request.model_state.pose.position.x = spawnx;
		model_state_srv_msg.request.model_state.pose.position.y = spawny;
		// model_state_srv_msg.request.model_state.pose.position.x = -21;
		// model_state_srv_msg.request.model_state.pose.position.y = -26;
		model_state_srv_msg.request.model_state.pose.position.z = 30;
		model_state_srv_msg.request.model_state.twist.linear.x  = 0.0;
		model_state_srv_msg.request.model_state.twist.linear.y  = 0.0;
		model_state_srv_msg.request.model_state.twist.linear.z  = -100;
		model_state_srv_msg.request.model_state.twist.angular.x = 0.0;
		model_state_srv_msg.request.model_state.twist.angular.y = 0.0;
		model_state_srv_msg.request.model_state.twist.angular.z = 0.0;
		model_state_srv_msg.request.model_state.reference_frame = "world";

		set_model_state_client.call(model_state_srv_msg);
		bool result = model_state_srv_msg.response.success;
		if (!result)
			ROS_WARN("service call to set_model_state failed!");

		bool shouldBail = false;
		// must finish spawn before time runs out or a respawn is needed
		time_t t1, t2;
		time(&t1);
		// wait for fresh reading from imu
		bool in_freefall = false;
		while (!in_freefall) {
			time(&t2);
			int diff = difftime(t2, t1);
			if (diff > SPAWN_TIME_LIMIT) {
				shouldBail = true;
				break;
			}
			ros::spinOnce();
			five_ms.sleep();
			ROS_INFO("waiting for imu to detect touchdown");
			double total_accel = abs(linear_acceleration.x) + abs(linear_acceleration.y)
			  + abs(linear_acceleration.z);
			if (total_accel < 5.0) {
				in_freefall = true;
				ROS_INFO("in freefall, accel: %f", total_accel);
			}
		}
		if (shouldBail) {
			continue;
		}

		bool touchdown = false;
		while (!touchdown) {
			time(&t2);
			int diff = difftime(t2, t1);
			if (diff > SPAWN_TIME_LIMIT) {
				shouldBail = true;
				break;
			}
			ros::spinOnce();
			five_ms.sleep();
			double total_accel = abs(linear_acceleration.x) + abs(linear_acceleration.y)
			  + abs(linear_acceleration.z);
			if (abs(total_accel) > 5.0) { // freefall accel is around 0, anything else should be like 9+
				touchdown = true;
				ROS_INFO("touchdown, accel: %f", total_accel);
			}
		}
		if (shouldBail) {
			continue;
		}

		// now that we've hit ground, spawn again with 0 velocity and random rotation.
		ros::spinOnce();
		gms_client.call(getmodelstate);

		double x = getmodelstate.response.pose.position.x;
		double y = getmodelstate.response.pose.position.y;
		double z = getmodelstate.response.pose.position.z;
		ROS_INFO("spawning based on: %f %f %f", x, y, z);
		model_state_srv_msg.request.model_state.pose.position.x = x;
		model_state_srv_msg.request.model_state.pose.position.y = y;
		model_state_srv_msg.request.model_state.pose.position.z = z + 0.75;

		model_state_srv_msg.request.model_state.twist.linear.z = 0.0;

		set_model_state_client.call(model_state_srv_msg);
		result = model_state_srv_msg.response.success;
		if (!result)
			ROS_WARN("service call to set_model_state failed2!");

		// allow for 3 attempts to orient the robot to a standing position
		bool spawnSuccessful = false;
		int i = 0;
		while (true) {
			contactFailure = false;
			// now wait for it to settle and check it upright
			bool settled = false;
			while (!settled) {
				time(&t2);
				int diff = difftime(t2, t1);
				if (diff > SPAWN_TIME_LIMIT) {
					shouldBail = true;
					break;
				}
				ros::spinOnce();
				five_ms.sleep();
				gms_client.call(getmodelstate);
				double total_linear = abs(getmodelstate.response.twist.linear.x)
				  + abs(getmodelstate.response.twist.linear.y)
				  + abs(getmodelstate.response.twist.linear.z);
				double total_angular = abs(getmodelstate.response.twist.angular.x)
				  + abs(getmodelstate.response.twist.angular.y)
				  + abs(getmodelstate.response.twist.angular.z);
				// need a 'high' linear velocity tolerance because of potential sliding on hills
				if (total_linear < 0.2 && total_angular < 0.025) {
					ROS_INFO("settling, lin, ang: %f %f", total_linear, total_angular);

					// need to be still enough for a period of time to count
					half_sec.sleep();
					ros::spinOnce();
					half_sec.sleep();
					ros::spinOnce();

					gms_client.call(getmodelstate);
					total_linear = abs(getmodelstate.response.twist.linear.x)
					  + abs(getmodelstate.response.twist.linear.y)
					  + abs(getmodelstate.response.twist.linear.z);
					total_angular = abs(getmodelstate.response.twist.angular.x)
					  + abs(getmodelstate.response.twist.angular.y)
					  + abs(getmodelstate.response.twist.angular.z);
					if (total_linear < 0.4 && total_angular < 0.08) {
						settled = true;
						ROS_INFO("fully settled, lin, ang: %f %f", total_linear, total_angular);
					} else {
						ROS_INFO("NOT settled, lin, ang: %f %f", total_linear, total_angular);
					}
				}
			}
			if (shouldBail) {
				break;
			}

			// now that it's not moving, check if upright and not in illegal contact, and rewspawn if not
			ros::spinOnce();
			gms_client.call(getmodelstate);
			if (checkIfUpwright(getmodelstate) && !contactFailure) {
				ROS_INFO("SPAWN SUCCESSFUL!");
				spawnSuccessful = true;
				break;
			} else if (i < 3) {
				gms_client.call(getmodelstate);
				x = getmodelstate.response.pose.position.x;
				y = getmodelstate.response.pose.position.y;
				z = getmodelstate.response.pose.position.z;
				ROS_INFO("retry #%d respawning based on: %f %f %f", i + 1, x, y, z);
				model_state_srv_msg.request.model_state.pose.position.x = x;
				model_state_srv_msg.request.model_state.pose.position.y = y;
				model_state_srv_msg.request.model_state.pose.position.z = z + 0.5;

				set_model_state_client.call(model_state_srv_msg);
				result = model_state_srv_msg.response.success;
				if (!result)
					ROS_WARN("service call to set_model_state failed3!");
			} else {
				ROS_WARN("SPAWN FAILED!!!! :'(");
				break;
			}
			i++;
		}

		if (shouldBail) {
			ROS_WARN("SPAWN TIMED OUT, respawning in new spot");
		}
		if (!spawnSuccessful) {
			continue; // skip to next loop iteration; ie spawn in a totally different spot.
		}
		shouldBail = false;

		// now start the drive and point cloud capture sequence
		gms_client.call(getmodelstate);
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,
		  getmodelstate.response.pose.orientation.y,
		  getmodelstate.response.pose.orientation.z,
		  getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		des_state_psi = yaw;
		// ROS_INFO("DESIRED YAW: %f", yaw);

		double carrot_dist = 10.0;
		double last_x      = getmodelstate.response.pose.position.x;
		double last_y      = getmodelstate.response.pose.position.y;
		// generate a 'carrot' for the robot, a point in front of it that it
		// will never actually reach because it keeps moving
		des_state_x = cos(des_state_psi) * carrot_dist + last_x;
		des_state_y = sin(des_state_psi) * carrot_dist + last_y;
		ROS_INFO("OG des_state_x, des_state_y: %f %f", des_state_x, des_state_y);
		ros::Rate sleep_timer(UPDATE_RATE);

		// timer for failure to progress
		time_t start, end;
		time(&start);

		while (true) {
			nonlinear_steering();

			gms_client.call(getmodelstate);
			double x = getmodelstate.response.pose.position.x;
			double y = getmodelstate.response.pose.position.y;
			// angle with respect to last position
			double robot_angle     = atan2(y - last_y, x - last_x);
			double angle_offset    = robot_angle - des_state_psi;
			double travel_dist     = sqrt(pow(last_x - x, 2) + pow(last_y - y, 2));
			double dist_along_path = cos(angle_offset) * travel_dist;
			double dist_from_path  = sin(angle_offset) * travel_dist;

			time(&end);
			int diff = difftime(end, start);
			// robot is considered stuck if it doesn't progress enough by the time limit
			if (diff > PROGRESS_TIME_LIMIT) {
				ROS_WARN("FAILURE DETECTED!!! - failure to progress, got stuck for too long");
				sendStopCmd();
				saveCroppedImages(x, y, false); // save with negative label
				break;
			}

			// boolean param tells it to save images if it finds a failure
			if (checkForPhysicalFailures(getmodelstate, true)) {
				break;
			}

			if (abs(dist_from_path) > widthBuffer) {
				// means part of the robot is not contained within the area of interest
				// See if it makes it back within a few seconds, if not count this as a failure
				ROS_WARN("Warning - slipped out of area of interest - waiting for return");
				// timer for failure to return to area of interest
				time_t start2, end2;
				double og_x = x;
				double og_y = y;

				bool failTheRobot     = true;
				bool shouldSaveImages = true;
				time(&start2);
				int diff = 0;
				while (diff < SLIPPAGE_TIME_LIMIT) {
					nonlinear_steering();
					gms_client.call(getmodelstate);
					// Robot is out of the are of interest so saving images would not result in good data
					// boolean param tells it NOT to save images if it finds a failure.
					if (checkForPhysicalFailures(getmodelstate, false)) {
						shouldSaveImages = false; // don't save, we can't reliably attribute this failure to an img
						break;
					}
					x = getmodelstate.response.pose.position.x;
					y = getmodelstate.response.pose.position.y;
					robot_angle     = atan2(y - last_y, x - last_x);
					angle_offset    = robot_angle - des_state_psi;
					travel_dist     = sqrt(pow(last_x - x, 2) + pow(last_y - y, 2));
					dist_along_path = cos(angle_offset) * travel_dist;
					dist_from_path  = sin(angle_offset) * travel_dist;
					if (abs(dist_from_path) < widthBuffer) {
						// made it back in time, so don't count this as a failure
						failTheRobot = false;
						ROS_WARN("Robot recovered from slippage :D");
						break;
					} else {
						time(&end2);
						diff = difftime(end2, start2);
					}
				}

				if (failTheRobot) {
					sendStopCmd();
					if (shouldSaveImages) {
						saveCroppedImages(og_x, og_y, false); // save with negative label
						ROS_WARN("FAILURE DETECTED!!! - failed to return to the area of interest in time");
					} else {
						ROS_WARN("FAILURE DETECTED!!! - physical failure while out of area of interest");
					}
					break;
				}
			}

			// take snapshot and move the carrot if we've move 0.3 meters or more down the line
			if (dist_along_path >= 0.3) { // TODO change back to 0.3
				// restart the timer since we've made it at least the desired distance
				// make sure we're facing foward pretty exactly, otherwise the captured image
				// will not be aligned with the desire line we're trying to drive on!
				tf::Quaternion q2(getmodelstate.response.pose.orientation.x,
				  getmodelstate.response.pose.orientation.y,
				  getmodelstate.response.pose.orientation.z,
				  getmodelstate.response.pose.orientation.w);
				tf::Matrix3x3 m(q2);
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);

				if (des_state_psi - yaw > CAPTURE_ANGLE_TOLERANCE) {
					// Skip the capturing part if the robot is not facing along the direction of travel very well
					// ROS_WARN("exceeded CAPTURE_ANGLE_TOLERANCE, skipping capture, angle is off by: %f",
					//   des_state_psi - yaw);
				} else {
					ROS_INFO("dist_along_path %f", dist_along_path);
					time(&start);
					shouldCapture = true;
					double xdiff = dist_along_path * cos(des_state_psi);
					double ydiff = dist_along_path * sin(des_state_psi);
					last_x     += xdiff;
					last_y     += ydiff;
					des_state_x = cos(des_state_psi) * carrot_dist + last_x;
					des_state_y = sin(des_state_psi) * carrot_dist + last_y;
					// now create and save the positively labeled images
					saveCroppedImages(x, y, true); // save with positive label
				}
			}
			ros::spinOnce();
		}
		// empty imagesStructVector so it can be filled again later with new data from the next run
		ROS_INFO("Clearing imagesStructVector");
		imagesStructVector.clear();


		// TODO maybe reject images that are more that XX% unkown (maybe 75%? think about cliffs)?
		// Or maybe its better for the ML algorithm to bias itself to guess unknowns base on proportion of
		// what its seen

		// have a conservative and yolo mode, ie safe for the physical robot vs what actually
		// makes the simualted robot fail

		// TODO need to test stairs vs hills, should say no for stairs, yes for
		// hills even if the slope in the same? 5cm pixel may be too large to differentiate...
		// try making the map smaller so that the stairs are the correct size?
	}
	return 0;
} // mainLoop

int main(int argc, char ** argv)
{
	ROS_INFO("Starting data collector");
	ros::init(argc, argv, "data_collection");
	ros::NodeHandle n;
	DataCollector dataCollector(&n);
	dataCollector.mainLoop();

	// should never get here
	return 0;
}
