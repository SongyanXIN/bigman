
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef FORWARD_COMMAND_CONTROLLER_FORWARD_JOINT_GROUP_COMMAND_CONTROLLER_H
#define FORWARD_COMMAND_CONTROLLER_FORWARD_JOINT_GROUP_COMMAND_CONTROLLER_H


#include <vector>
#include <string>
#include <urdf/model.h>
#include <angles/angles.h>
#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid_gains_setter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <custom_effort_controllers/JointCommand.h>
#include <custom_effort_controllers/CommandArrayStamped.h>

namespace custom_effort_controllers
{

/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the effort command signal down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "custom_effort_controllers::JointGroupPositionEffortController"
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (custom_effort_controllers::CommandArrayStamped) : The joint commands to apply.
 */

class JointGroupPositionEffortController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

    /**
    * \brief Store position and velocity command in struct to allow easier realtime buffer usage
    */
    struct Command
    {
        double position_;
        double velocity_;
        double torque_;
        bool has_velocity_;
    };

    JointGroupPositionEffortController() : loop_count_ ( 0 ) {}
    ~JointGroupPositionEffortController()
    {
        sub_command_.shutdown();
    }


    bool init ( hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n )
    {
        // Get list of controlled joints
        if ( !n.getParam ( "joints", joint_names_ ) )
        {
            ROS_ERROR_STREAM ( "Failed to getParam 'joints' (namespace: " << n.getNamespace() << ")." );
            return false;
        }

        // Get number of joints
        n_joints_ = joint_names_.size();
        if ( n_joints_ == 0 )
        {
            ROS_ERROR_STREAM ( "List of joint names is empty." );
            return false;
        }

        // Load PID Controller using gains set on parameter server
        if ( !n.getParam ( "ps", ps_ ) )
        {
            ROS_ERROR_STREAM ( "Failed to getParam 'ps' (namespace: " << n.getNamespace() << ")." );
            return false;
        }
        if ( !n.getParam ( "is", is_ ) )
        {
            ROS_ERROR_STREAM ( "Failed to getParam 'is' (namespace: " << n.getNamespace() << ")." );
            return false;
        }
        if ( !n.getParam ( "ds", ds_ ) )
        {
            ROS_ERROR_STREAM ( "Failed to getParam 'ds' (namespace: " << n.getNamespace() << ")." );
            return false;
        }


        // init pid controllers
        for ( unsigned int i=0; i < n_joints_; ++i )
        {
            pid_controllers_.push_back ( control_toolbox::Pid ( ps_[i],is_[i],ds_[i],0.0,0.0 ) );
            //pid_controllers_[i].initDynamicReconfig ( n );
        }

        // init pid controller gain setters
        for ( unsigned int i=0; i < n_joints_; ++i )
        {
            pid_gain_setters_.push_back ( control_toolbox::PidGainsSetter() );
        }

        for ( unsigned int i=0; i < n_joints_; ++i )
        {
            pid_gain_setters_[i].add ( &pid_controllers_[i] );
            pid_gain_setters_[i].advertise ( joint_names_[i] );
        }




        // Start realtime state publisher
        // to be done...

        // Start command subscriber
        sub_command_ = n.subscribe<custom_effort_controllers::CommandArrayStamped>( "command", 1, &JointGroupPositionEffortController::CommandCB, this );

        // Get joints handles from hardware interface
        for ( unsigned int i=0; i<n_joints_; i++ )
        {
            joints_.push_back ( robot->getHandle ( joint_names_[i] ) );
        }

        // Get URDF info about joints
        urdf::Model urdf;
        if ( !urdf.initParam ( "robot_description" ) )
        {
            ROS_ERROR ( "Failed to parse urdf file" );
            return false;
        }
        for ( unsigned int i=0; i < n_joints_; ++i )
        {
            joint_urdfs_.push_back ( urdf.getJoint ( joint_names_[i] ) );
        }


        return true;
    }





    void starting ( const ros::Time& time )
    {

        // get current joints position
        std::vector <double> pos_commands;
        std::vector <double> vel_commands;
        std::vector <double> tor_commands;


        for ( unsigned int i=0; i<n_joints_; ++i )
        {
            pos_commands.push_back ( joints_[i].getPosition() );
            vel_commands.push_back ( joints_[i].getVelocity() );
            tor_commands.push_back ( joints_[i].getEffort() );
        }

        // Make sure joints are within limits if applicable
        for ( unsigned int i=0; i<n_joints_; ++i )
        {
            enforceJointLimits ( pos_commands[i], joint_urdfs_[i] );
        }


        
//         for ( unsigned int i=0; i<n_joints_; ++i ){
// 	    name_to_position_.insert ( std::pair<std::string, double> ( joint_names_[i],pos_commands[i] ) ); 
// 	    name_to_velocity_.insert ( std::pair<std::string, double> ( joint_names_[i],vel_commands[i] ) );
// 	    name_to_torque_.insert ( std::pair<std::string, double> ( joint_names_[i],tor_commands[i] ) );
// 	    name_to_has_velocity_.insert ( std::pair<std::string, bool> ( joint_names_[i],false ) );
// 	}
//         
//         name_to_position_buffer_.writeFromNonRT(name_to_position_);
// 	name_to_velocity_buffer_.writeFromNonRT(name_to_velocity_);
// 	name_to_torque_buffer_.writeFromNonRT(name_to_torque_);
// 	name_to_has_velocity_buffer_.writeFromNonRT(name_to_has_velocity_);
        
        
        
        
//         for ( unsigned int i=0; i<n_joints_; ++i )
//         {
//             name_vec_		[i] =  joint_names_[i];
//             position_vec_	[i] =  pos_commands[i];
//             velocity_vec_	[i] =  vel_commands[i];
//             torque_vec_		[i] =  tor_commands[i];
//             has_velocity_vec_	[i] =  false;
// 
//         }
// 
//         name_vec_buffer_.writeFromNonRT ( name_vec_ );
//         position_vec_buffer_.writeFromNonRT ( position_vec_ );
//         velocity_vec_buffer_.writeFromNonRT ( velocity_vec_ );
//         torque_vec_buffer_.writeFromNonRT ( torque_vec_ );
//         has_velocity_vec_buffer_.writeFromNonRT ( has_velocity_vec_ );





        for ( unsigned int i=0; i<n_joints_; ++i )
        {
            Command temp_command = {  pos_commands[i],
                                      vel_commands[i],
                                      tor_commands[i],
                                      false
                                   };

            command_structs_.push_back ( temp_command );


        }

        command_buffers_.initRT ( command_structs_ );

        for ( unsigned int i=0; i<n_joints_; ++i )
        {
            pid_controllers_[i].reset();
        }


    }








    void update ( const ros::Time& time, const ros::Duration& period )
    {



        command_structs_ = * ( command_buffers_.readFromRT() );

//         name_vec_ = * ( name_vec_buffer_.readFromRT() );
//         position_vec_ = * ( position_vec_buffer_.readFromRT() );
//         velocity_vec_ = * ( velocity_vec_buffer_.readFromRT() );
//         torque_vec_ = * ( torque_vec_buffer_.readFromRT() );
//         has_velocity_vec_ = * ( has_velocity_vec_buffer_.readFromRT() );
	
// 	  name_to_position_     = *(name_to_position_buffer_.readFromRT() );
// 	  name_to_velocity_     = *(name_to_velocity_buffer_.readFromRT() );
// 	  name_to_torque_       = *(name_to_torque_buffer_.readFromRT() );
// 	  name_to_has_velocity_ = *(name_to_has_velocity_buffer_.readFromRT() );
// 	

        for ( unsigned int i=0; i<n_joints_; i++ )
        {
            double command_position = command_structs_[i].position_     ;
            double command_velocity = command_structs_[i].velocity_     ;
            double command_torque   = command_structs_[i].torque_       ;
            bool   has_velocity_    = command_structs_[i].has_velocity_ ;

//             double command_position = position_vec_[i]     ;
//             double command_velocity = velocity_vec_[i]     ;
//             double command_torque   = torque_vec_[i]      ;
//             bool   has_velocity_    = has_velocity_vec_[i] ;
	  
//             double command_position = name_to_position_[joint_names_[i]]     ;
//             double command_velocity = name_to_velocity_[joint_names_[i]]     ;
//             double command_torque   = name_to_torque_[joint_names_[i]]       ;
//             bool   has_velocity_    = name_to_has_velocity_[joint_names_[i]] ;
	  
	  
// 


            double error, vel_error;
            double feedback_effort, feedforward_effort;
            double commanded_effort;

            double current_position = joints_[i].getPosition();

            boost::shared_ptr<const urdf::Joint> joint_urdf_ = joint_urdfs_[i];
            enforceJointLimits ( command_position, joint_urdfs_[i] );

            // Compute position error
            if ( joint_urdf_->type == urdf::Joint::REVOLUTE )
            {
                angles::shortest_angular_distance_with_limits (
                    current_position,
                    command_position,
                    joint_urdf_->limits->lower,
                    joint_urdf_->limits->upper,
                    error );
            }
            else if ( joint_urdf_->type == urdf::Joint::CONTINUOUS )
            {
                error = angles::shortest_angular_distance ( current_position, command_position );
            }
            else //prismatic
            {
                error = command_position - current_position;
            }


            // Decide which of the two PID computeCommand() methods to call
            if ( has_velocity_ )
            {
                // Compute velocity error if a non-zero velocity command was given
                vel_error = command_velocity - joints_[i].getVelocity();

                // Set the PID error and compute the PID command with nonuniform
                // time step size. This also allows the user to pass in a precomputed derivative error.
                feedback_effort = pid_controllers_[i].computeCommand ( error, vel_error, period );
            }
            else
            {
                // Set the PID error and compute the PID command with nonuniform
                // time step size.
                feedback_effort = pid_controllers_[i].computeCommand ( error, period );
            }

            // torque feedforward part
            feedforward_effort = command_torque;
            commanded_effort = feedback_effort + feedforward_effort;
            joints_[i].setCommand ( commanded_effort );

        }


    }




    std::vector< std::string > joint_names_;
    std::vector< hardware_interface::JointHandle > joints_;
    std::vector< boost::shared_ptr<const urdf::Joint> > joint_urdfs_;
    unsigned int n_joints_;
    
    // solusion 1    
    std::vector< Command > command_structs_; // pre-allocated memory that is re-used to set the realtime buffer
    realtime_tools::RealtimeBuffer<std::vector< Command > > command_buffers_;


//     //solusion 2
//     // realtime buffer
//     realtime_tools::RealtimeBuffer< std::vector<std::string> > 	name_vec_buffer_;
//     realtime_tools::RealtimeBuffer< std::vector<double> > 	position_vec_buffer_;
//     realtime_tools::RealtimeBuffer< std::vector<double> > 	velocity_vec_buffer_;
//     realtime_tools::RealtimeBuffer< std::vector<double> > 	torque_vec_buffer_;
//     realtime_tools::RealtimeBuffer< std::vector<bool> > 	has_velocity_vec_buffer_;
// 
//     // pre-allocated memory that is re-used to set the realtime buffer
//     std::vector<std::string> 	name_vec_;
//     std::vector<double> 	position_vec_;
//     std::vector<double> 	velocity_vec_;
//     std::vector<double> 	torque_vec_;
//     std::vector<bool> 		has_velocity_vec_;

    
    
    
    
//     // pre-allocated memory that is re-used to set the realtime buffer
//     std::map<std::string, double> name_to_position_;
//     std::map<std::string, double> name_to_velocity_;
//     std::map<std::string, double> name_to_torque_;
//     std::map<std::string, bool>   name_to_has_velocity_;
//     
//     realtime_tools::RealtimeBuffer< std::map<std::string, double> > 	name_to_position_buffer_;
//     realtime_tools::RealtimeBuffer< std::map<std::string, double> > 	name_to_velocity_buffer_;
//     realtime_tools::RealtimeBuffer< std::map<std::string, double> > 	name_to_torque_buffer_;
//     realtime_tools::RealtimeBuffer< std::map<std::string, bool> > 	name_to_has_velocity_buffer_;

private:
    int loop_count_;
    std::vector< double > ps_;
    std::vector< double > is_;
    std::vector< double > ds_;
    std::vector< control_toolbox::Pid > pid_controllers_;       /**< Internal PID controller. */
    std::vector< control_toolbox::PidGainsSetter > pid_gain_setters_;   /* open services for setting pid gains */

    ros::Subscriber sub_command_;


    void CommandCB ( const custom_effort_controllers::CommandArrayStampedConstPtr& msg )
    {
        // make map from joint command msg
//         for ( unsigned int i=0; i<msg->name.size(); ++i )
//         {
//             name_to_position_.insert ( std::pair<std::string, double> ( msg->name[i],msg->position[i] ) );
//             name_to_velocity_.insert ( std::pair<std::string, double> ( msg->name[i],msg->velocity[i] ) );
//             name_to_torque_.insert ( std::pair<std::string, double> ( msg->name[i],msg->torque[i] ) );
// 	    name_to_has_velocity_.insert ( std::pair<std::string, bool> ( msg->name[i],msg->has_velocity[i] ) );
//         }
// 
//         name_to_position_buffer_.writeFromNonRT(name_to_position_);
// 	name_to_velocity_buffer_.writeFromNonRT(name_to_velocity_);
// 	name_to_torque_buffer_.writeFromNonRT(name_to_torque_);
// 	name_to_has_velocity_buffer_.writeFromNonRT(name_to_has_velocity_);
  

//         for ( unsigned int i=0; i<msg->name.size(); ++i )
//         {
//             name_vec_[i] = msg->name[i];
//             position_vec_[i] = msg->position[i];
//             velocity_vec_[i] = msg->velocity[i];
//             torque_vec_[i] = msg->torque[i];
//             has_velocity_vec_[i] = msg->has_velocity[i];
//         }
// 
// 
//         name_vec_buffer_.writeFromNonRT ( name_vec_ );
//         position_vec_buffer_.writeFromNonRT ( position_vec_ );
//         velocity_vec_buffer_.writeFromNonRT ( velocity_vec_ );
//         torque_vec_buffer_.writeFromNonRT ( torque_vec_ );
//         has_velocity_vec_buffer_.writeFromNonRT ( has_velocity_vec_ );



    for (unsigned int i=0; i<n_joints_; ++i)
    {
      command_structs_[i].position_     = msg->commands[i].position	;
      command_structs_[i].velocity_     = msg->commands[i].velocity	;
      command_structs_[i].torque_       = msg->commands[i].torque	;
      command_structs_[i].has_velocity_ = msg->commands[i].has_velocity	;

    }

    command_buffers_.writeFromNonRT(command_structs_);

    }


    void enforceJointLimits ( double &command_position, boost::shared_ptr<const urdf::Joint> &joint_urdf_ )
    {
        // Check that this joint has applicable limits
        if ( joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC )
        {
            if ( command_position > joint_urdf_->limits->upper ) // above upper limnit
            {
                command_position = joint_urdf_->limits->upper;
            }
            else if ( command_position < joint_urdf_->limits->lower ) // below lower limit
            {
                command_position = joint_urdf_->limits->lower;
            }
        }
    }


};

}

#endif









