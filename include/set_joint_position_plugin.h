#ifndef MY_ROBOT_PLUGIN_HH
#define MY_ROBOT_PLUGIN_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
// #include <RobotLib/RobotLib.h>

#include "gazebo/transport/transport.hh"




namespace gazebo
{


class set_joint_position_plugin : public ModelPlugin
{
    /// \brief Constructor
public: set_joint_position_plugin();

    /// \brief Destructor
public: virtual ~set_joint_position_plugin();

    // Documentation inherited
protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
protected: virtual void UpdateChild();

    /// \brief call back when a Wrench message is published
    /// \param[in] _msg The Incoming ROS message representing the new force to exert.
private: void CallBackMethod(const sensor_msgs::JointState _msg);

    /// \brief The custom callback queue thread function.
private: void QueueThread();

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
private: ros::NodeHandle* rosnode_;
private: ros::Subscriber sub_;
    sensor_msgs::JointState joint_state_;

private: void PublishJointState();

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
private: boost::mutex lock_;

    /// \brief ROS JointState topic name inputs
private: std::string topic_name_;

    /// \brief for setting ROS name space
private: std::string robot_namespace_;

    // Custom Callback Queue
private: ros::CallbackQueue queue_;
    /// \brief Thead object for the running callback Thread.
private: boost::thread callback_queue_thread_;
    // Pointer to the update event connection
private: event::ConnectionPtr update_connection_;


private: std::vector<physics::JointPtr> joints_list;

private: sensor_msgs::JointState set_joint_state_;



};
}
#endif
