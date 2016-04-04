
#include <algorithm>
#include <assert.h>

#include <set_joint_position_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(set_joint_position_plugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
set_joint_position_plugin::set_joint_position_plugin()
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
set_joint_position_plugin::~set_joint_position_plugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  this->joints_list.clear();
  this->sub_.shutdown();


  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void set_joint_position_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robot_namespace_ = "";

  if ( _sdf->HasElement("robotNamespace") ) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";


  if ( !_sdf->HasElement("topicName") )
  {
    ROS_FATAL("force plugin missing <topicName>, cannot proceed");
    return;
  }
  else this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if ( !ros::isInitialized() )
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_STREAM("topicName:" << this->topic_name_);
  ROS_INFO_STREAM("Loading plugin for setting joint positions !:");

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


  this->sub_ = this->rosnode_->subscribe( this->topic_name_, 1, &set_joint_position_plugin::CallBackMethod, this, ros::TransportHints().tcpNoDelay() );


  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind(&set_joint_position_plugin::QueueThread, this) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&set_joint_position_plugin::UpdateChild, this) );


  /* Checking the list of joints */
  this->joints_list = _model->GetJoints();

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void set_joint_position_plugin::CallBackMethod(const sensor_msgs::JointState _msg)
{
  this->lock_.lock();
  ROS_INFO_STREAM( "Getting a new position" );
  set_joint_state_ =  _msg;
  this->lock_.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void set_joint_position_plugin::UpdateChild()
{
  this->lock_.lock();

  for (int i = 0; i < set_joint_state_.position.size();i++){
     #if GAZEBO_MAJOR_VERSION == 2
        this->joints_list[i]->SetAngle(0, set_joint_state_.position[i]);   
    #else // if GAZEBO_MAJOR_VERSION != 2
        this->joints_list[i]->SetPosition(0, set_joint_state_.position[i]);
    #endif // if GAZEBO_MAJOR_VERSION == 2
  }

  this->lock_.unlock();

}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void set_joint_position_plugin::QueueThread()
{
  static const double timeout = 0.01;

  while ( this->rosnode_->ok() )
  {
    this->queue_.callAvailable( ros::WallDuration(timeout) );
  }
}


}
