#ifndef PUB_DES_STATE_H_
#define PUB_DES_STATE_H_

#include <queue>
#include <stack>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mapping_and_control/path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


//constants and parameters:
const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz


//FOR SIMULATION
// const double accel_max = 0.5; //1m/sec^2
// const double alpha_max = 1.0; // rad/sec^2
// const double speed_max = 1.0; //1 m/sec
// const double omega_max = 3.0; //1 rad/sec

//FOR REAL ROBOT
const double accel_max = 0.5; //1m/sec^2
const double alpha_max = 1.0; // rad/sec^2
const double speed_max = 0.3; //1 m/sec
const double omega_max = 0.5; //1 rad/sec

const double path_move_tol = 0.08;  // if path points are within 2cm, close enough

//max distance the robot can travel before adding a new point to the return path
const double return_path_point_spacing = 2.0;
//the max delta angle the robot can turn before adding a new point to the return path
const double return_path_delta_phi = 0.02; //bout 2 degrees

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;
const int OFF = 4;

class DesStatePublisher {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    nav_msgs::Odometry current_state_;

    //some class member variables:
    nav_msgs::Path path_;
    std::vector<nav_msgs::Odometry> des_state_vec_;
    nav_msgs::Odometry des_state_;
    nav_msgs::Odometry halt_state_;
    nav_msgs::Odometry seg_end_state_;
    nav_msgs::Odometry seg_start_state_;
    nav_msgs::Odometry current_des_state_;
    nav_msgs::Odometry current_vel_state;
    geometry_msgs::Twist halt_twist_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped end_pose_;
    geometry_msgs::PoseStamped current_pose_;
    std_msgs::Float64 float_msg_;
    double des_psi_;
    std::queue<geometry_msgs::PoseStamped> path_queue_; //a C++ "queue" object, stores vertices as Pose points in a FIFO queue
    std::stack<geometry_msgs::PoseStamped> return_path_stack; // so we can find our way home, LIFO queue
    int motion_mode_;
    bool e_stop_trigger_; //these are intended to enable e-stop via a service
    bool e_stop_reset_;
    int traj_pt_i_;
    int npts_traj_;
    double dt_;
    //dynamic parameters: should be tuned for target system
    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_; 

    // some objects to support service and publisher
    ros::ServiceServer estop_service_;
    ros::ServiceServer estop_clear_service_;
    ros::ServiceServer lidar_alarm_service_;
    ros::ServiceServer flush_path_queue_;
    ros::ServiceServer pop_path_queue_;
    ros::ServiceServer append_path_;
    
    ros::Publisher desired_state_publisher_;
    ros::Publisher des_psi_publisher_;
    ros::Publisher motion_mode_publisher_;
    
    //last time we got a transform
    double lastUpdate;

	tf::StampedTransform odom_to_map;

    //a trajectory-builder object; 
    TrajBuilder trajBuilder_; 

    tf::TransformBroadcaster br_;
    tf::StampedTransform stfBaseLinkWrtOdom_;

    tf::TransformListener tfListener;

    ros::Subscriber odom_subscriber_;
    ros::Subscriber cmd_mode_subscriber_;
    ros::Subscriber go_home_subscriber_;

    ros::Subscriber tf_subscriber_;
    //geometry_msgs::Transform drift_correct_transform;


    // member methods:
    void updateTransform();

    void initializePublishers();
    void initializeServices();
    bool estopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool clearEstopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool lidarAlarmServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool flushPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool popPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool appendPathQueueCB(mapping_and_control::pathRequest& request,mapping_and_control::pathResponse& response);

    //toMap is true if you want to convert from odom to map, false if want map to odom
    geometry_msgs::PoseStamped get_corrected_des_state(geometry_msgs::PoseStamped uncorrectedPoseStamped, bool toMap);

    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void cmdModeCallback(const std_msgs::Int32& message_holder);
    void goHomeRobotYoureDrunk(const std_msgs::Int32& message_holder);
    // void tfCallback(const tf2_msgs::TFMessage& tf_message);

public:
    DesStatePublisher(ros::NodeHandle& nh);//constructor
    bool lidar_alarm; 
    bool h_e_stop_;
    int get_motion_mode() {return motion_mode_;}
    void set_motion_mode(int mode) {motion_mode_ = mode;}
    bool get_estop_trigger() { return e_stop_trigger_;}
    void reset_estop_trigger() { e_stop_trigger_ = false;}
    void set_init_pose(double x,double y, double psi);
    void pub_next_state();
    void append_path_queue(geometry_msgs::PoseStamped pose) { path_queue_.push(pose); }
    void append_path_queue(double x, double y, double psi) 
        { path_queue_.push(trajBuilder_.xyPsi2PoseStamped(x,y,psi)); }
};
#endif
