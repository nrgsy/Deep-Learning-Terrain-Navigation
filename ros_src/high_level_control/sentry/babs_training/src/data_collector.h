#ifndef DATA_COLLECTOR_H_
#define DATA_COLLECTOR_H_

// TODO REALLY need to get rid of some of these includes, no way are they all needed anymore

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <boost/filesystem.hpp>
#include <vector>
#include <iostream>
#include <time.h>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <gazebo_msgs/ModelStates.h>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ContactsState.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;

// robot is a 0.762 m wide (.368 body + 2*.197 treads)
const double footprintWidth = 0.76;
// 1.22 m long (1.068 body + .152 lidar box)
const double bodyLen      = 1.068;
const double lidarLen     = .152;
const double footprintLen = bodyLen + lidarLen;
const double widthBuffer  = 0.27;
const double frontBuffer  = 0.48;
// lookahead distance in meters
const double stripLen    = 4.0;                              // TODO change back to 4.0
const double targetWidth = footprintWidth + 2 * widthBuffer; // = 1.3
const double targetLen   = footprintLen + frontBuffer;       // = 1.7
// be sure the strip divides evenly into pixels with this resoltuion
const double resolution   = 0.05;
const double frontSpacing = 0.5;

const double UPDATE_RATE  = 100.0;
const double TARGET_SPEED = 0.5; // m/sec
const double MAX_BOOST    = 2.5; // m/sec, how much it can exceed the TARGET_SPEED
// m/sec, robot is considered out of control and a failure if this speed is exceeded
const double FAILURE_SPEED = 7.0;
const double MAX_OMEGA     = 4.0;                // rad/sec
const double CAPTURE_ANGLE_TOLERANCE = 0.0436;   // radians, (2.5 degrees)
const double K_PSI = 5.0;                        // control gains for steering, default was 5.0
const double K_LAT_ERR_THRESH = 2 * widthBuffer; // deafult was 3.0;

// char PATH_PREFIX[] = "/media/thomas/MasterDrive/training_data/";
// char PATH_PREFIX[] = "/media/thomas/MasterDrive/training_data2/";
// char PATH_PREFIX[] = "./";
char PATH_PREFIX[] = "/media/thomas/MasterDrive/6/";

// char PATH_PREFIX[]      = "/home/thomas/Desktop/thesis_partial/labeled_data/training_data/";
double SPAWN_TIME_LIMIT    = 30; // SECONDS
double SLIPPAGE_TIME_LIMIT = 5;  // SECONDS
double PROGRESS_TIME_LIMIT = 15; // SECONDS

// (0.76 + 2*0.27)/0.05 = 26 and (1.22 + 0.48)/0.05 = 34
// so the resulting images from the target area will have a resolution of 26x34

class DataCollector
{
public:
	DataCollector(ros::NodeHandle * nodehandle);
	int mainLoop();
private:
	ros::NodeHandle n;

	ros::Subscriber imu_subscriber;
	ros::Subscriber pcl_subscriber;
	ros::Subscriber contact_subscriber;
	ros::Publisher cropped_cloud_pub;
	ros::Publisher cmd_publisher;
	ros::ServiceClient gms_client;
	gazebo_msgs::GetModelState getmodelstate;
	ros::ServiceClient set_model_state_client;
	geometry_msgs::Quaternion imu_orientation;
	geometry_msgs::Vector3 linear_acceleration;

	bool contactFailure;
	bool shouldCapture;
	bool imuReady;
	bool outputPositive;
	int fileCounter;

	ros::Duration half_sec;
	ros::Duration five_ms;

	double des_state_x;
	double des_state_y;
	double des_state_psi;
	double des_state_speed;
	double des_state_omega;

	struct ImageStruct {
		double x, y;
		int ** imageMatrix;
	};

	vector<ImageStruct> imagesStructVector;

	void saveCroppedImages(double xcoord, double ycoord, bool label);

	void saveImage(int ** imageMatrix, int xdim, int ydim, char valuesFile[], char imageFile[]);
	void take_snapshot(const sensor_msgs::PointCloud2& input);
	bool checkIfUpwright(gazebo_msgs::GetModelState gms);
	void pointCloudCallback(const sensor_msgs::PointCloud2& cloud);
	void lidarBumperCallback(const gazebo_msgs::ContactsState& state);
	void imuCallback(const sensor_msgs::Imu& imu_in);
	double omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path);
	double psi_strategy(double offset_err);
	void nonlinear_steering();
	double sat(double x);
	double min_dang(double dang);
	void sendStopCmd();
	bool checkForPhysicalFailures(gazebo_msgs::GetModelState gms, bool shouldSaveImages);
};

#endif // ifndef DATA_COLLECTOR_H_
