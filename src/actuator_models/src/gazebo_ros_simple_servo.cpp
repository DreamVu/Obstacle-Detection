#include <algorithm>
#include <assert.h>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <ignition/math/Vector3.hh>
#include "nowtech_actuator_models/gazebo_ros_simple_servo.h"

namespace gazebo {

// Constructor
GazeboRosSimpleServo::GazeboRosSimpleServo() {
}

// Destructor
GazeboRosSimpleServo::~GazeboRosSimpleServo() {
	FiniChild();
}

double GazeboRosSimpleServo::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

void GazeboRosSimpleServo::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "ServoPlugin" ) );
    gazebo_ros_->isInitialized();
    gazebo_ros_->getParameter<double> ( update_rate_, 				"update_rate", 100.0 );
		gazebo_ros_->getParameter<double> ( actuator_velocity_, 	"actuator_velocity", 0.1 );
		gazebo_ros_->getParameter<double> ( actuator_accuracy_, 	"actuator_accuracy", 0.01 );
		gazebo_ros_->getParameter<double> ( actuator_fmax_, 			"actuator_fmax", 10.0 );
		gazebo_ros_->getParameter<double> ( actuator_fudge_, 			"actuator_fudge", 1.0 );
		gazebo_ros_->getParameterBoolean  ( aux_controller_, 			"aux_controller", false );
		gazebo_ros_->getParameter<std::string> ( command_topic_,  "command_topic",  "/servo/command" );
		gazebo_ros_->getParameter<std::string> ( feedback_topic_, "feedback_topic",  "/servo/position" );
		gazebo_ros_->getParameter<std::string> ( servo_name_,  		"servo_name",  		"servo_joint" );
		gazebo_ros_->getParameter<int> ( axis_index_, 	"axis_index", 0 );

		// TODO: check if joint is not null
    joint_ = gazebo_ros_->getJoint ( parent, "actuated_joint", "servo_joint" );
		joint_lower_limit_ = joint_->LowerLimit();
		ROS_INFO_NAMED(servo_name_, "%s LowerLimit: %f ", servo_name_.c_str(), joint_lower_limit_);
		joint_upper_limit_ = joint_->UpperLimit();
		ROS_INFO_NAMED(servo_name_, "%s UpperLimit: %f ", servo_name_.c_str(), joint_upper_limit_);
		joint_->SetParam( "fmax", 0, actuator_fmax_ );
		joint_->SetParam( "fudge_factor", 0, actuator_fudge_);

		joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_state", 1000);

		if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_; else this->update_period_ = 0.0;
		last_update_time_ = parent->GetWorld()->SimTime();

	  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32> (
        command_topic_,
				1,
        boost::bind(&GazeboRosSimpleServo::commandCallback, this, _1),
        ros::VoidPtr(),
        &queue_
    );
    command_subscriber_ = gazebo_ros_->node()->subscribe(so);

    this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosSimpleServo::QueueThread, this ) );
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosSimpleServo::UpdateChild, this ) );

		if (this->aux_controller_){
			positon_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(feedback_topic_, 1);
	    ROS_INFO_NAMED(servo_name_, "%s: Advertising position feedback on %s ", gazebo_ros_->info(), feedback_topic_.c_str());
		}

		current_position_ = joint_->Position ( axis_index_ );
		desired_position_ = current_position_;
}

void GazeboRosSimpleServo::Reset() {
  last_update_time_ = parent->GetWorld()->SimTime();
}

void GazeboRosSimpleServo::publishJointState() {
  ros::Time current_time = ros::Time::now();
  joint_state_.header.stamp = current_time;
  joint_state_.name.resize ( 1 );
  joint_state_.position.resize ( 1 );
  joint_state_.name[0] = this->joint_->GetName();
  joint_state_.position[0] = current_position_;
  joint_state_publisher_.publish ( joint_state_ );
}

// Plugin update function
void GazeboRosSimpleServo::UpdateChild() {
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

		current_position_ = joint_->Position ( axis_index_ );

		if (this->aux_controller_)
		{
			std_msgs::Float32 position_msg;
			position_msg.data = current_position_;
			positon_publisher_.publish(position_msg);
			joint_->SetParam( "fmax", 0, actuator_fmax_ );
			joint_->SetParam( "vel",  0, velocity_command_ );
			if (current_position_ > joint_upper_limit_) ROS_WARN_NAMED(servo_name_,
					"%s over upper limit: %f ", servo_name_.c_str(), joint_upper_limit_);
			if (current_position_ < joint_lower_limit_) ROS_WARN_NAMED(servo_name_,
					"%s over lower limit: %f ", servo_name_.c_str(), joint_lower_limit_);			
		}
		else
		{ // simple ideal controller
			distance_ = desired_position_ - current_position_;
			if (fabs(distance_) > actuator_accuracy_){
				velocity_command_ = distance_ * actuator_velocity_;
			}
			else {
				velocity_command_ = 0.0;
			}
			joint_->SetParam( "fmax", 0, actuator_fmax_ );
			joint_->SetParam( "vel",  0, velocity_command_ );
		}

		// publish joint states regularly
    if ( seconds_since_last_update > update_period_ ) {
				publishJointState();
				last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosSimpleServo::FiniChild() {
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

// Callback from custom que
void GazeboRosSimpleServo::commandCallback ( const std_msgs::Float32::ConstPtr& cmd_msg ) {

	if (this->aux_controller_)
	{
		velocity_command_ = cmd_msg->data;
	}
	else
	{
    desired_position_ = cmd_msg->data;
		if (desired_position_ < joint_lower_limit_) {
			desired_position_ = joint_lower_limit_;
			ROS_WARN_NAMED(servo_name_, "%s Constraining command to lower limit: %f ", servo_name_.c_str(), joint_lower_limit_);
		}
		if (desired_position_ > joint_upper_limit_) {
			desired_position_ = joint_upper_limit_;
			ROS_WARN_NAMED(servo_name_, "%s Constraining command to upper limit: %f ", servo_name_.c_str(), joint_upper_limit_);
		}
	}
}

void GazeboRosSimpleServo::QueueThread() {
    static const double timeout = 1.0/update_rate_;
    while ( gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosSimpleServo )
// eof_ns
}
