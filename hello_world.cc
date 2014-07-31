#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>


#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

ach_channel_t channel;
double my_msg[9]={0};
//double my_msg_size;
size_t my_msg_size;

namespace gazebo
{
 

class hello_world : public ModelPlugin
  {
   


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
    {
     
enum ach_status r = ach_create( "my_channel", 10, 512, NULL );
if( ACH_OK != r ) {
fprintf( stderr, "Could not create channel: %s\n", ach_result_to_string(r) );
exit(EXIT_FAILURE);
}



r = ach_open( &channel, "my_channel", NULL );
if( ACH_OK != r ) {
    syslog( LOG_ERR, "Could not open channel: %s\n", ach_result_to_string(r) );
    exit(EXIT_FAILURE);
}

      // Store the pointer to the model
      this->model = _parent;

      this->j2_controller = new physics::JointController(model);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&hello_world::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      
size_t frame_size;
enum ach_status r = ach_get( &channel, &my_msg, sizeof(my_msg), &frame_size, NULL, 0 );
if( ACH_MISSED_FRAME == r ) {
    fprintf(stdout, "Missed a/some messages(s)\n");
} else if( ACH_STALE_FRAMES == r ) {
    fprintf( stdout, "No new data\n" );
} else if( ACH_OK != r ) {
    syslog( LOG_ERR, "Unable to get a message: %s\n", ach_result_to_string(r) );
}
    
      double angle(1);
      //std::string j2name("joint1");   
      j2_controller->SetJointPosition(this->model->GetJoint("joint1"),  my_msg[0]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint2"),  my_msg[1]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint3"),  my_msg[2]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint4"),  my_msg[3]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint5"),  my_msg[4]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint6"),  my_msg[5]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint7"),  my_msg[6]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint8"),  my_msg[7]);
      j2_controller->SetJointPosition(this->model->GetJoint("joint9"),  my_msg[8]);
      

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointController * j2_controller;
    //enum ach_status ach_close(ach_channel_t *channel);
    //enum ach_status ach_unlink(const char *name);

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(hello_world)
}
