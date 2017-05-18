#ifndef DRIVER_H_
#define DRIVER_H_

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
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
// #include <pcl/visualization/pcl_visualizer.h>

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

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ctime>

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
const double stripLen    = 4.0;
const double targetWidth = footprintWidth + 2 * widthBuffer; // = 1.3
const double targetLen   = footprintLen + frontBuffer;       // = 1.7
// be sure the strip divides evenly into pixels with this resoltuion
const double resolution   = 0.05;
const double frontSpacing = 0.5;

const double TARGET_SPEED = 0.4; // m/sec
const double MAX_BOOST    = 0.4; // m/sec, how much it can exceed the TARGET_SPEED
// m/sec, robot is considered out of control and a failure if this speed is exceeded
const double FAILURE_SPEED = 7.0;

const double TARGET_OMEGA    = 0.3;                      // rad/sec
const double MAX_OMEGA       = 4.0;                      // rad/sec
const double MAX_OMEGA_BOOST = MAX_OMEGA - TARGET_OMEGA; // rad/sec

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
double VOXEL_LEAF_SIZE     = 0.04;

static const int NUM_AREAS  = 24; // NUM_AREAS/2 = the number in each of the two circles
static const int NUM_IMAGES = 1;  // number of images predicted at a time
double predictionMatrix[NUM_AREAS][NUM_IMAGES];
double DELTA_PSI = 2 * M_PI / (NUM_AREAS / 2);
int front_index  = NUM_AREAS / 8; // index in predictionMatrix corresponding to the front rectangle

// should be between 0 and 1, closer to 1 means the robot will care more about
// the last prediction, closer to 0 it'll care more about previous predictions
double memory_decay_rate = 0.5;

void mySigintHandler(int sig);
int sockfd;

// (0.76 + 2*0.27)/0.05 = 26 and (1.22 + 0.48)/0.05 = 34
// so the resulting images from the target area will have a resolution of 26x34

class Driver
{
public:
	Driver(ros::NodeHandle * nodehandle);
	int Go();
private:
	ros::NodeHandle n;

	ros::Subscriber imu_subscriber;
	ros::Subscriber pcl_subscriber;
	ros::Subscriber pcl_subscriber1;
	ros::Subscriber pcl_subscriber2;
	ros::Subscriber pcl_subscriber3;
	ros::Subscriber contact_subscriber;
	ros::Publisher cropped_cloud_pub1;
	ros::Publisher cropped_cloud_pub2;
	ros::Publisher cropped_cloud_pub3;
	ros::Publisher cropped_cloud_pub4;
	ros::Publisher voxel_cloud_pub;
	ros::Publisher cmd_publisher;
	ros::ServiceClient gms_client;
	ros::ServiceClient set_model_state_client;

	gazebo_msgs::GetModelState getmodelstate;
	geometry_msgs::Quaternion imu_orientation;
	geometry_msgs::Vector3 linear_acceleration;
	sensor_msgs::PointCloud2 kinectcloud;
	sensor_msgs::PointCloud2 kinectcloud1;
	sensor_msgs::PointCloud2 kinectcloud2;
	sensor_msgs::PointCloud2 kinectcloud3;

	ros::Duration half_sec;
	ros::Duration five_ms;
	ros::Duration one_ms;

	bool contactFailure;
	bool shouldProcess;
	bool imuReady;
	bool outputPositive;
	bool merging_cloud;
	bool firstCloudSeen;
	bool firstCloudProcessed;

	int fileCounter;

	double des_state_x;
	double des_state_y;
	double des_state_psi;
	double des_state_speed;
	double des_state_omega;
	double prediction_memory;

	struct ImageStruct {
		double x, y;
		int ** imageMatrix;
	};

	void process_cloud(const sensor_msgs::PointCloud2& input);

	void pointCloudCallback(const sensor_msgs::PointCloud2& cloud);
	void pointCloudCallback1(const sensor_msgs::PointCloud2& cloud);
	void pointCloudCallback2(const sensor_msgs::PointCloud2& cloud);
	void pointCloudCallback3(const sensor_msgs::PointCloud2& cloud);

	void imuCallback(const sensor_msgs::Imu& imu_in);
	double omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path);
	double psi_strategy(double offset_err);
	void nonlinear_steering();
	double sat(double x);
	double min_dang(double dang);
	void stop();
	void sendRotateCmd(double direction);
	double getRobotYaw();
	void voxelFilter(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& voxel_out);
	void transformYAxis(double yaw, pcl::PointCloud<pcl::PointXYZ>& cloud,
	  pcl::PointCloud<pcl::PointXYZ>& transformed_cloud, double xtrans, double ztrans);
};

#endif // ifndef DATA_COLLECTOR_H_
