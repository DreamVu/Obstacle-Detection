#ifndef _LIGHT_PLUGIN_H_
#define _LIGHT_PLUGIN_H_

#include <memory>
#include <string>

// Gazebo
#include <ignition/math/Color.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

  class FlashLightSettingPrivate;
  class GZ_PLUGIN_VISIBLE FlashLightSetting {

  public:

    FlashLightSetting(
    const sdf::ElementPtr &_sdf,
    const physics::ModelPtr &_model,
    const common::Time &_currentTime);

    virtual ~FlashLightSetting();
    virtual void InitPubLight(const transport::PublisherPtr &_pubLight) final;
    virtual void UpdateLightInEnv(const common::Time &_currentTime) final;
    virtual const std::string Name() const final;
    virtual const physics::LinkPtr Link() const final;
    virtual void SwitchOn() final;
    virtual void SwitchOff() final;
    virtual void SetDuration(const double _duration, const int _index) final;
    virtual void SetDuration(const double _duration) final;
    virtual void SetInterval(const double _interval, const int _index) final;
    virtual void SetInterval(const double _interval) final;
    virtual void SetColor(const ignition::math::Color &_color, const int _index) final;
    virtual void SetColor(const ignition::math::Color &_color) final;
    virtual unsigned int BlockCount() final;
    virtual bool RemoveBlock(const int _index) final;
    virtual void InsertBlock(
    const double _duration, const double _interval,
    const ignition::math::Color &_color, const int _index) final;

  protected:

    virtual void Flash();
    virtual void Dim();
    virtual ignition::math::Color CurrentColor() final;

  private:
    std::unique_ptr<FlashLightSettingPrivate> dataPtr;

  };

  class FlashLightPluginPrivate;
  class GZ_PLUGIN_VISIBLE FlashLightPlugin : public ModelPlugin
  {
    public:
      FlashLightPlugin();
      virtual ~FlashLightPlugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

    protected:
      virtual void OnUpdate();

    private:
      GazeboRosPtr gazebo_ros_;
      event::ConnectionPtr update_connection_;
      ros::Subscriber light_control_subscriber_;
      std::string command_topic_;
      ros::CallbackQueue queue_;
      std::thread callback_queue_thread_;
      boost::mutex lock_;
      std::unique_ptr<FlashLightPluginPrivate> dataPtr;
      void cmdCallback(const std_msgs::Bool::ConstPtr& cmd_msg);
      void QueueThread();

    protected:
      virtual bool TurnOn(const std::string &_lightName) final;
      virtual bool TurnOn(const std::string &_lightName, const std::string &_linkName) final;
      virtual bool TurnOnAll() final;
      virtual bool TurnOff(const std::string &_lightName) final;
      virtual bool TurnOff(const std::string &_lightName, const std::string &_linkName) final;
      virtual bool TurnOffAll() final;
      virtual bool ChangeDuration(
        const std::string &_lightName, const std::string &_linkName,
        const double _duration, const int _index) final;
      virtual bool ChangeDuration(
        const std::string &_lightName, const std::string &_linkName,
        const double _duration) final;
      virtual bool ChangeInterval(
        const std::string &_lightName, const std::string &_linkName,
        const double _interval, const int _index) final;
      virtual bool ChangeInterval(
        const std::string &_lightName, const std::string &_linkName,
        const double _interval) final;
      virtual bool ChangeColor(
        const std::string &_lightName, const std::string &_linkName,
        const ignition::math::Color &_color, const int _index) final;
      virtual bool ChangeColor(
        const std::string &_lightName, const std::string &_linkName,
        const ignition::math::Color &_color) final;
      virtual std::shared_ptr<FlashLightSetting> CreateSetting(
        const sdf::ElementPtr &_sdf,
        const physics::ModelPtr &_model,
        const common::Time &_currentTime);
      virtual void InitSettingBySpecificData(
        std::shared_ptr<FlashLightSetting> &_setting);

  };
}


#endif
