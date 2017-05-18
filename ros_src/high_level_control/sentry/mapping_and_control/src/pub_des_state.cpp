
#include <mapping_and_control/pub_des_state.h>
//ExampleRosClass::ExampleRosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)

DesStatePublisher::DesStatePublisher(ros::NodeHandle& nh) : nh_(nh) {

	//as_(nh, "pub_des_state_server", boost::bind(&DesStatePublisher::executeCB, this, _1),false) {
	//as_.start(); //start the server running
	//configure the trajectory builder:
	//dt_ = dt; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
	trajBuilder_.set_dt(dt);
	//dynamic parameters: should be tuned for target system
	accel_max_ = accel_max;
	trajBuilder_.set_accel_max(accel_max_);
	alpha_max_ = alpha_max;
	trajBuilder_.set_alpha_max(alpha_max_);
	speed_max_ = speed_max;
	trajBuilder_.set_speed_max(speed_max_);
	omega_max_ = omega_max;
	trajBuilder_.set_omega_max(omega_max_);
	path_move_tol_ = path_move_tol;
	trajBuilder_.set_path_move_tol_(path_move_tol_);
	initializePublishers();
	initializeServices();
	//define a halt state; zero speed and spin, and fill with viable coords
	halt_twist_.linear.x = 0.0;
	halt_twist_.linear.y = 0.0;
	halt_twist_.linear.z = 0.0;
	halt_twist_.angular.x = 0.0;
	halt_twist_.angular.y = 0.0;
	halt_twist_.angular.z = 0.0;
	motion_mode_ = OFF; //init in state ready to process new goal
	h_e_stop_ = false;
	e_stop_trigger_ = false; //these are intended to enable e-stop via a service
	e_stop_reset_ = false; //and reset estop
	current_pose_ = trajBuilder_.xyPsi2PoseStamped(0,0,0);
	start_pose_ = current_pose_;
	end_pose_ = current_pose_;
	current_des_state_.twist.twist = halt_twist_;
	current_des_state_.pose.pose = current_pose_.pose;
	halt_state_ = current_des_state_;
	seg_start_state_ = current_des_state_;
	seg_end_state_ = current_des_state_;
	lidar_alarm = false;

	//init the drift transform
	// drift_correct_transform.translation.x = 0.0;
	// drift_correct_transform.translation.y = 0.0;
	// drift_correct_transform.rotation = trajBuilder_.convertPlanarPsi2Quaternion(0.0);

	odom_subscriber_ = nh_.subscribe("/odom", 1, &DesStatePublisher::odomCallback, this); //subscribe to odom messages
	cmd_mode_subscriber_ = nh_.subscribe("/cmd_mode", 1, &DesStatePublisher::cmdModeCallback, this);
	go_home_subscriber_ = nh_.subscribe("/go_home", 1, &DesStatePublisher::goHomeRobotYoureDrunk, this);
	// tf_subscriber_ = nh_.subscribe("/tf", 1, &DesStatePublisher::tfCallback, this);

	lastUpdate = -10;

}

void DesStatePublisher::initializeServices() {
	ROS_INFO("Initializing Services");
	estop_service_ = nh_.advertiseService("estop_service",
			&DesStatePublisher::estopServiceCallback, this);
	estop_clear_service_ = nh_.advertiseService("clear_estop_service",
			&DesStatePublisher::clearEstopServiceCallback, this);
	lidar_alarm_service_ = nh_.advertiseService("lidar_alarm_service",
			&DesStatePublisher::lidarAlarmServiceCallback, this);
	pop_path_queue_ = nh_.advertiseService("pop_path_queue_service",
			&DesStatePublisher::popPathQueueCB, this);
	flush_path_queue_ = nh_.advertiseService("flush_path_queue_service",
			&DesStatePublisher::flushPathQueueCB, this);
	append_path_ = nh_.advertiseService("append_path_queue_service",
			&DesStatePublisher::appendPathQueueCB, this);

}

//member helper function to set up publishers;

void DesStatePublisher::initializePublishers() {
	ROS_INFO("Initializing Publishers");
	desired_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desState", 1, true);
	des_psi_publisher_ = nh_.advertise<std_msgs::Float64>("/desPsi", 1);
	motion_mode_publisher_ = nh_.advertise<std_msgs::Int32>("/motion_mode", 1);
}

bool DesStatePublisher::estopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	ROS_WARN("estop!!");
	e_stop_trigger_ = true;
	return true;
}

bool DesStatePublisher::clearEstopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	ROS_INFO("estop reset");
	e_stop_reset_ = true;
	return true;
}

bool DesStatePublisher::lidarAlarmServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	ROS_WARN("lidar_alarm!!");
	lidar_alarm = true;
	return true;
}

bool DesStatePublisher::flushPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	ROS_WARN("flushing path queue");
	while (!path_queue_.empty()) {
		path_queue_.pop();
	}
	return true;
}

bool DesStatePublisher::popPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
	ROS_INFO("removing element from path queue");
	path_queue_.pop();
	return true;
}

bool DesStatePublisher::appendPathQueueCB(mapping_and_control::pathRequest& request, mapping_and_control::pathResponse& response) {

	int npts = request.path.poses.size();
	ROS_INFO("appending path queue with %d points", npts);
	for (int i = 0; i < npts; i++) {

		updateTransform();
		path_queue_.push(request.path.poses[i]); //should be coming in as map coordinates already
	}
	return true;
}

void DesStatePublisher::odomCallback(const nav_msgs::Odometry& odom_rcvd) { 

	current_state_ = odom_rcvd;

	geometry_msgs::Pose odom_pose_ = odom_rcvd.pose.pose;
	geometry_msgs::Quaternion odom_quat_ = odom_rcvd.pose.pose.orientation;
	double odom_phi_ = trajBuilder_.convertPlanarQuat2Psi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
	tf::Vector3 pos;
	pos.setX(odom_pose_.position.x);
	pos.setY(odom_pose_.position.y);
	pos.setZ(odom_pose_.position.z);

	stfBaseLinkWrtOdom_.stamp_ = ros::Time::now();
	stfBaseLinkWrtOdom_.setOrigin(pos);

	tf::Quaternion q;
	q.setX(odom_quat_.x);
	q.setY(odom_quat_.y);
	q.setZ(odom_quat_.z);
	q.setW(odom_quat_.w);

	stfBaseLinkWrtOdom_.setRotation(q);

	stfBaseLinkWrtOdom_.frame_id_ = "odom";
	stfBaseLinkWrtOdom_.child_frame_id_ = "base_link";

	br_.sendTransform(stfBaseLinkWrtOdom_);
}

void DesStatePublisher::cmdModeCallback(const std_msgs::Int32& message_holder) {

	if (message_holder.data != 0 && message_holder.data != 1) {
		ROS_WARN("Got unexpected motion_mode_ from cmd_mode topic");
	}
	else if (message_holder.data == 0) {
		ROS_INFO("motion_mode_ set to OFF!");
		motion_mode_ = OFF;
	}
	else if (message_holder.data == 1 && motion_mode_ == OFF) {
		ROS_INFO("motion_mode_ set to DONE_W_SUBGOAL!");
		motion_mode_ = DONE_W_SUBGOAL;
	}

}

void DesStatePublisher::goHomeRobotYoureDrunk(const std_msgs::Int32& message_holder) {

	if (motion_mode_ != OFF) {

		ROS_WARN("Robot going home");
		while (!path_queue_.empty()) {
			path_queue_.pop();
		}

		while (!return_path_stack.empty()) {

			geometry_msgs::PoseStamped top = return_path_stack.top();

			ROS_WARN("moved map(%f,%f) from path stack to path queue",top.pose.position.x,top.pose.position.y);

			path_queue_.push(top);
			return_path_stack.pop();

		}
	}
	else {
		ROS_WARN("Robot not drunk enough to go home");
	}
    std::stack<geometry_msgs::PoseStamped> newStack;
    return_path_stack = newStack;


}

// void DesStatePublisher::tfCallback(const tf2_msgs::TFMessage& tf_message) {

//     for (geometry_msgs::TransformStamped &transformStamped : tf_message.transforms) {
//         ROS_WARN("GOT A TransformStamped");
//         if (transformStamped.child_frame_id == "odom" && transformStamped.header.frame_id_ = "map") {
//             ROS_WARN("EYYY Got the right TransformStamped");
//             drift_correct_transform = transformStamped.transform;
//         }
//     }
// }

void DesStatePublisher::set_init_pose(double x, double y, double psi) {
	current_pose_ = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
}

//here is a state machine to advance desired-state publications
// this will cause a single desired state to be published
// The state machine advances through modes, including
// HALTING, E_STOPPED, DONE_W_SUBGOAL, and PURSUING_SUBGOAL
// does PURSUING_SUBGOAL->DONE_W_SUBGOAL->PURSUING_SUBGOAL
// or HALTING->E_STOPPED->DONE_W_SUBGOAL->PURSUING_SUBGOAL
// transition to HALTING requires triggering an e-stop via service estop_service_
// transition from HALTING to E_STOPPED occurs with completing of halt trajectory
// transition from E_STOPPED to DONE_W_SUBGOAL requires estop reset via 
//   service estop_clear_service_
// transition DONE_W_SUBGOAL->PURSUING_SUBGOAL depends on at least one path
//   point in the queue path_queue_
// transition PURSUING_SUBGOAL->DONE_W_SUBGOAL occurs when current subgoal has
//   been reached
// path queue can be flushed via service flush_path_queue_,
// or points can be appended to path queue w/ service append_path_

void DesStatePublisher::pub_next_state() {

	//skip if in off mode
	if (!OFF) {
		// first test if an e-stop has been triggered
		if (e_stop_trigger_ && ((motion_mode_!=HALTING) && (motion_mode_!=E_STOPPED))) {
			ROS_WARN("E-STOP TRIGGERED");
			e_stop_trigger_ = false; //reset trigger
			//compute a halt trajectory
			trajBuilder_.build_braking_traj(current_pose_, des_state_vec_,current_vel_state);
			motion_mode_ = HALTING;
			traj_pt_i_ = 0;
			npts_traj_ = des_state_vec_.size();
		}

		if (h_e_stop_ && ((motion_mode_!=HALTING) && (motion_mode_!=E_STOPPED))) {
			ROS_WARN("HARDWARE E-STOP TRIGGERED");
			h_e_stop_ = false; //reset trigger
			//compute a halt trajectory
			trajBuilder_.build_braking_traj(current_pose_, des_state_vec_,current_vel_state);
			motion_mode_ = HALTING;
			traj_pt_i_ = 0;
			npts_traj_ = des_state_vec_.size();
		}

		if (lidar_alarm && ((motion_mode_!=HALTING) && (motion_mode_!=E_STOPPED))) {
			ROS_WARN("LIDAR ALARM TRIGGERED");
			lidar_alarm = false; //reset trigger
			//compute a halt trajectory
			trajBuilder_.build_braking_traj(current_pose_, des_state_vec_,current_vel_state);
			motion_mode_ = HALTING;
			traj_pt_i_ = 0;
			npts_traj_ = des_state_vec_.size();
		}

		//or if an e-stop has been cleared
		if (e_stop_reset_) {
			e_stop_reset_ = false; //reset trigger
			if (motion_mode_ != E_STOPPED) {
				ROS_WARN("e-stop reset while not in e-stop mode");
			}            //OK...want to resume motion from e-stopped mode;
			else {
				motion_mode_ = DONE_W_SUBGOAL; //this will pick up where left off
				ROS_INFO("DONE WITH SUBGOAL");
			}
		}
	}

	//state machine; results in publishing a new desired state
	switch (motion_mode_) {
	case OFF: //occurs when robot is switched to you the controller instead of this
		//constantly update current pose
		current_pose_.pose = current_state_.pose.pose;
		current_pose_.header = current_state_.header;

		seg_end_state_ = current_state_;
		//publish whatever current state is as desired state
		desired_state_publisher_.publish(current_state_);
		break;
	case E_STOPPED: //this state must be reset by a service
		desired_state_publisher_.publish(halt_state_);
		break;

	case HALTING: //e-stop service callback sets this mode
		//if need to brake from e-stop, service will have computed
		// new des_state_vec_, set indices and set motion mode;
		ROS_INFO("HALTING");
		current_des_state_ = des_state_vec_[traj_pt_i_];
		current_des_state_.header.stamp = ros::Time::now();
		desired_state_publisher_.publish(current_des_state_);

		current_pose_.pose = current_des_state_.pose.pose;
		current_pose_.header = current_des_state_.header;
		des_psi_ = trajBuilder_.convertPlanarQuat2Psi(current_pose_.pose.orientation);
		float_msg_.data = des_psi_;
		des_psi_publisher_.publish(float_msg_);

		traj_pt_i_++;
		//segue from braking to halted e-stop state;
		if (traj_pt_i_ >= npts_traj_) { //here if completed all pts of braking traj
			halt_state_ = des_state_vec_.back(); //last point of halting traj
			// make sure it has 0 twist
			halt_state_.twist.twist = halt_twist_;
			seg_end_state_ = halt_state_;
			current_des_state_ = seg_end_state_;
			motion_mode_ = E_STOPPED; //change state to remain halted
			ROS_INFO("FINISHED HALT, now E_STOPPED");
		}
		break;

	case PURSUING_SUBGOAL: //if have remaining pts in computed traj, send them
		//extract the i'th point of our plan:
		//ROS_INFO("PURSUING SUBGOAL");
		current_des_state_ = des_state_vec_[traj_pt_i_];
		current_vel_state = current_des_state_;
		current_pose_.pose = current_des_state_.pose.pose;
		current_des_state_.header.stamp = ros::Time::now();
		desired_state_publisher_.publish(current_des_state_);
		//next three lines just for convenience--convert to heading and publish
		// for rqt_plot visualization
		des_psi_ = trajBuilder_.convertPlanarQuat2Psi(current_pose_.pose.orientation);
		float_msg_.data = des_psi_;
		des_psi_publisher_.publish(float_msg_);
		traj_pt_i_++; // increment counter to prep for next point of plan
		//check if we have clocked out all of our planned states:
		if (traj_pt_i_ >= npts_traj_) {
			motion_mode_ = DONE_W_SUBGOAL; //if so, indicate we are done
			seg_end_state_ = des_state_vec_.back(); // last state of traj

			geometry_msgs::PoseStamped poseToAdd;
			poseToAdd.pose = current_state_.pose.pose;
			poseToAdd.header = current_state_.header;
			//			geometry_msgs::PoseStamped topPoseInStack = get_corrected_des_state(return_path_stack.top(),false);
			//			//geometry_msgs::PoseStamped topPoseInStack = return_path_stack.top();
			//			double currentX = current_state_.pose.pose.position.x;
			//			double currentY = current_state_.pose.pose.position.y;
			//			double lastX = topPoseInStack.pose.position.x;
			//			double lastY = topPoseInStack.pose.position.y;
			//			double dist = sqrt(pow(lastX - currentX,2) + pow(lastY - currentY,2));  //thanks pythagoras, you da man
			//			double current_psi = trajBuilder_.convertPlanarQuat2Psi(current_state_.pose.pose.orientation);
			//			double last_psi = trajBuilder_.convertPlanarQuat2Psi(topPoseInStack.pose.orientation);
			//			double psiDiff = abs(current_psi - last_psi);
			//			//ROS_INFO("(currentX, currentY, lastX, lastY,current_psi,last_psi = %f, %f, %f, %f, %f, %f)",currentX, currentY, lastX, lastY,current_psi,last_psi);
			//			//ROS_INFO("(dist >= return_path_point_spacing) && (psiDiff >= return_path_delta_phi) = (%f >= %f) && (%f >= %f)",dist,return_path_point_spacing,psiDiff,return_path_delta_phi);
			//			if (dist >= return_path_point_spacing || psiDiff >= return_path_delta_phi) {
			ROS_INFO("return_path_stack got a point");
			return_path_stack.push(get_corrected_des_state(poseToAdd,true));
			//return_path_stack.push(poseToAdd);
			//			}
			ROS_INFO("DONE WITH SUBGOAL: x = %f, y= %f", path_queue_.front().pose.position.x, path_queue_.front().pose.position.y);
			if (!path_queue_.empty()) {
				path_queue_.pop(); // done w/ this subgoal; remove from the queue
			}
		}
		break;

	case DONE_W_SUBGOAL: //suspended, pending a new subgoal
		//see if there is another subgoal is in queue; if so, use
		//it to compute a new trajectory and change motion mode

		if (return_path_stack.empty()) {
			ROS_INFO("return_path_stack got its first point");
			geometry_msgs::PoseStamped poseToAdd;
			poseToAdd.pose = current_state_.pose.pose;
			poseToAdd.header = current_state_.header;
			return_path_stack.push(get_corrected_des_state(poseToAdd,true));
		}

		if (!path_queue_.empty()) {
			updateTransform();

			//ROS_WARN("DONE 1");

			int n_path_pts = path_queue_.size();
			ROS_INFO("%d points in path queue", n_path_pts);
			start_pose_ = current_pose_;

			end_pose_ = get_corrected_des_state(path_queue_.front(),false);
			//end_pose_ = path_queue_.front();


			trajBuilder_.build_point_and_go_traj(start_pose_, end_pose_, des_state_vec_);
			traj_pt_i_ = 0;
			npts_traj_ = des_state_vec_.size();
			motion_mode_ = PURSUING_SUBGOAL; // got a new plan; change mode to pursue it
			ROS_INFO("PURSUING SUBGOAL: x = %f, y= %f", path_queue_.front().pose.position.x, path_queue_.front().pose.position.y);

		} else { //no new goal? stay halted in this mode

			//ROS_WARN("DONE 2, current_state_ x,y = (%f,%f)", current_state_.pose.pose.position.x, current_state_.pose.pose.position.y);
			//ROS_WARN("DONE 2, seg_end_state_ x,y = (%f,%f)", seg_end_state_.pose.pose.position.x, seg_end_state_.pose.pose.position.y);

			// by simply reiterating the last state sent (should have zero vel)
			desired_state_publisher_.publish(seg_end_state_);
		}
		break;

	default: //this should not happen
		ROS_WARN("motion mode not recognized!");
		desired_state_publisher_.publish(current_des_state_);
		break;
	}

	//publish the desired motion_mode_
	std_msgs::Int32 msg;
	msg.data = motion_mode_;
	motion_mode_publisher_.publish(msg);

}

void DesStatePublisher::updateTransform() {

	//    if (tfListener.canTransform("map","odom",uncorrectedPoseStamped.header.stamp)) {
	//	if (tfListener.waitForTransform("map","odom",ros::Time(0), ros::Duration(3.0))) {
	if (tfListener.canTransform("map","odom",ros::Time(0))) {

		bool failure = true;
		ROS_INFO("EYYY can transform, waiting for pose transform...");
		while (failure) {
			failure = false;
			try {
				//tfListener.transformPose("map",uncorrectedPoseStamped,correctedPoseStamped);

				//   tfListener.transformPose("map",uncorrectedPoseStamped.header.stamp,uncorrectedPoseStamped,"odom",correctedPoseStamped);

				tfListener.lookupTransform("map", "odom", ros::Time(0), odom_to_map);

			} catch (tf::TransformException &exception) {
				ROS_WARN("%s; retrying lookup!!!", exception.what());
				failure = true;
				ros::Duration(0.5).sleep(); // sleep for half a second
			}
		}

		double x = odom_to_map.getOrigin().x();
		double y = odom_to_map.getOrigin().y();
		double psi = odom_to_map.getRotation().getAngle();

		ROS_INFO("transform (x,y,psi) is (%f,%f,%f)",x,y,psi);
	}
	else {
		ROS_WARN("TEARS can't transform");
	}

}

geometry_msgs::PoseStamped DesStatePublisher::get_corrected_des_state(geometry_msgs::PoseStamped uncorrectedPoseStamped, bool toMap) {

	double x = odom_to_map.getOrigin().x();
	double y = odom_to_map.getOrigin().y();
	double psi = odom_to_map.getRotation().getAngle();

	double now = ros::Time::now().toSec();

	//	if (now - lastUpdate > 5.0) {
	//
	//		//ROS_INFO("TRYING TO CORRECT... x,y before: %f, %f", uncorrectedPoseStamped.pose.position.x, uncorrectedPoseStamped.pose.position.y);
	//
	//		x = odom_to_map.getOrigin().x();
	//		y = odom_to_map.getOrigin().y();
	//		psi = odom_to_map.getRotation().getAngle();
	//
	//		ROS_INFO("transform (x,y,psi) is (%f,%f,%f)",x,y,psi);
	//
	//		now = ros::Time::now().toSec();
	//		lastUpdate = now;
	//	}


	geometry_msgs::PoseStamped correctedPoseStamped = uncorrectedPoseStamped;

	double ogPsi = trajBuilder_.convertPlanarQuat2Psi(correctedPoseStamped.pose.orientation);

	if (toMap) {
		correctedPoseStamped.pose.position.x += x;
		correctedPoseStamped.pose.position.y += y;
		correctedPoseStamped.pose.orientation = trajBuilder_.convertPlanarPsi2Quaternion(ogPsi + psi);
	}
	else {
		correctedPoseStamped.pose.position.x -= x;
		correctedPoseStamped.pose.position.y -= y;
		correctedPoseStamped.pose.orientation = trajBuilder_.convertPlanarPsi2Quaternion(ogPsi - psi);
	}

	//ROS_INFO("AFTER: x,y %f, %f", correctedPoseStamped.pose.position.x, correctedPoseStamped.pose.position.y);

	return correctedPoseStamped;



	//return uncorrectedPoseStamped;


	// double driftX = drift_correct_transform.translation.x;
	// double driftY = drift_correct_transform.translation.y;
	// double drift_psi = trajBuilder_.convertPlanarQuat2Psi(drift_correct_transform.rotation);

	// double stateX = uncorrectedState.pose.pose.position.x;
	// double stateY = uncorrectedState.pose.pose.position.y;
	// double state_psi = trajBuilder_.convertPlanarQuat2Psi(uncorrectedState.pose.pose.orientation);


	// correctedState.pose.pose.position.x =;
	// correctedState.pose.pose.position.y =;

	// correctedState.pose.pose.orientation = trajBuilder_.convertPlanarPsi2Quaternion(0.0);

	// subtract?


	// correctedState.


}
