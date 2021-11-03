#ifndef _IDEAL_ACTUATOR_PLUGIN_H_
#define _IDEAL_ACTUATOR_PLUGIN_H_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosSimpleServo : public ModelPlugin {

    public:

      GazeboRosSimpleServo();
      ~GazeboRosSimpleServo();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:

      virtual void UpdateChild();
      virtual void FiniChild();

    private:

      GazeboRosPtr gazebo_ros_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      physics::JointPtr joint_;
      ros::Publisher  joint_state_publisher_;
      ros::Publisher  positon_publisher_;
      ros::Subscriber command_subscriber_;
      sensor_msgs::JointState joint_state_;
      std::string command_topic_;
      std::string feedback_topic_;
      std::string servo_name_;
      double update_rate_;
      double joint_lower_limit_;
      double joint_upper_limit_;
      double current_position_;
      double desired_position_;
      double distance_;
      double velocity_command_;
      double actuator_velocity_;
      double actuator_accuracy_;
      double actuator_fmax_;
      double actuator_fudge_;
      int axis_index_;
      bool aux_controller_;

      // Callback Queue
      ros::CallbackQueue queue_;
      std::thread callback_queue_thread_;
      boost::mutex lock_;
      void QueueThread();

      // Helper variables
      double update_period_;
      common::Time last_update_time_;

      double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
      void commandCallback(const std_msgs::Float32::ConstPtr& cmd_msg);
      void publishJointState();
  };


} // namespace

#endif
