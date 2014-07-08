/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser
// Edited: Steffen Pfiffner
// Purpose: Interface from moveit gripper_command to jaco api.

#include "jaco/gripper_controller.h"

namespace kinova
{

 GripperAction::GripperAction(boost::shared_ptr<AbstractJaco> jaco) :
    node_(ros::NodeHandle()),
    jaco_(jaco),
    action_server_(node_, "jaco_gripper_controller/gripper_command",
                   boost::bind(&GripperAction::goalCB, this, _1),
                   boost::bind(&GripperAction::cancelCB, this, _1), true),
    has_active_goal_(false)
  {
	 ros::NodeHandle pn("~");


     pn.param("goal_position_threshold", goal_position_threshold_, 0.1);
     pn.param("goal_effort_threshold", goal_effort_threshold_, 0.05);
     pn.param("stall_velocity_threshold", stall_velocity_threshold_, 1e-6);
     pn.param("stall_timeout", stall_timeout_, 5.0);

     ROS_INFO("Gripper Controller started");
  }

  GripperAction::~GripperAction()
  {
	 pub_controller_command_.shutdown();
     sub_controller_state_.shutdown();

  }


  void GripperAction::goalCB(GoalHandle gh)
  {
	  // Cancels the currently active goal.

      if (has_active_goal_)
      {
    	  // Marks the current goal as canceled.
    	  active_goal_.setCanceled();
    	  has_active_goal_ = false;
          std::cout << "canceled current active goal" << std::endl;
      }

      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;
      goal_received_ = ros::Time::now();
      min_error_seen_ = 1e10;

      ROS_INFO_STREAM("Gripper target position: " << active_goal_.getGoal()->command.position << "effort: " << active_goal_.getGoal()->command.max_effort);

      ROS_INFO("setFingersValues");
    
      std::vector<double> fingerPositionsRadian = jaco_->getFingersJointAngle();

      std::cout << "current fingerpositions: " << fingerPositionsRadian[0] << " " << fingerPositionsRadian[1] << " " << fingerPositionsRadian[2] << std::endl;

        target_position = active_goal_.getGoal()->command.position;
        target_effort = active_goal_.getGoal()->command.max_effort;

        //determine if the gripper is supposed to be opened or closed
        double current_position = jaco_->getFingersJointAngle()[0];
        if(current_position > target_position){
            opening = true;
            std::cout << "Opening gripper" << std::endl;
        }else{
            opening = false;
            std::cout << "Closing gripper" << std::endl;
        }

        

        fingerPositionsRadian[0] = target_position;
        fingerPositionsRadian[1] = target_position;
        fingerPositionsRadian[2] = target_position;

        

        std::cout << "Gripper target position: " << active_goal_.getGoal()->command.position << "effort: " << active_goal_.getGoal()->command.max_effort << std::endl;


      double fingerPositionsDegree[3] = {radToDeg(fingerPositionsRadian[0]),radToDeg(fingerPositionsRadian[1]),radToDeg(fingerPositionsRadian[2])};

      if(!jaco_->setFingersValues(fingerPositionsDegree))
      {
    	  active_goal_.setCanceled();
    	  ROS_ERROR("Cancelling goal: moveJoint didn't work.");
          std::cout << "Cancelling goal because setFingers didn't work" << std::endl;
      }

      last_movement_time_ = ros::Time::now();
  }

  void GripperAction::cancelCB(GoalHandle gh)
  {
	  if (active_goal_ == gh)
      {

          // Marks the current goal as canceled.
		  active_goal_.setCanceled();
		  has_active_goal_ = false;

          jaco_->stop();
      }
  }

  double GripperAction::radToDeg(double rad)
  {
	  return rad*57.295779513;
  }

  void GripperAction::update()
  {

        if(has_active_goal_){

            //here only the position of finger one is used            
            double current_position = jaco_->getFingersJointAngle()[0];

            //add the currents of all fingers together to form the current effort
            double current_effort = jaco_->getFingersCurrent()[0] + jaco_->getFingersCurrent()[1] + jaco_->getFingersCurrent()[2];

                control_msgs::GripperCommandResult result;
              result.position = current_position;
              result.effort = current_effort;
              result.reached_goal = false;
              result.stalled = false;

            //while opening the gripper the position is used to determine if the grasp was successful, if closing the effort value is used. 

            if(opening){

                if(fabs(current_position - target_position) < goal_position_threshold_){
                    
                      jaco_->stop();

                      result.reached_goal = true;
                      active_goal_.setSucceeded(result);
                      has_active_goal_ = false;

                      std::cout << "Gripper command successful!" << std::endl;

                      return;
                }
                                   
            //closing
            }else{

                 std::cout << "Current finger effort " << current_effort << std::endl;

                 if(current_effort > target_effort){
 
                      //TODO: Add check for no object

                      jaco_->stop();

                      result.reached_goal = true;
                      active_goal_.setSucceeded(result);
                      has_active_goal_ = false;

                      std::cout << "Gripper command successful!" << std::endl;

                      return;
                }

            }
                
            if((ros::Time::now() - last_movement_time_).toSec() > stall_timeout_){

    		  result.stalled = true;
    		  active_goal_.setAborted(result);
    		  has_active_goal_ = false;

              std::cout << "Grasp failed because stalled.." << std::endl;

              return;

           


            }

        }

  }

/*
  void GripperAction::controlStateCB(const control_msgs::JointControllerStateConstPtr &msg)
  {
	  last_controller_state_ = msg;
      ros::Time now = ros::Time::now();

      if (!has_active_goal_)
    	  return;

       std::cout << "set point: " << msg->set_point << std::endl;

      // Ensures that the controller is tracking my setpoint.
      if (fabs(msg->set_point - active_goal_.getGoal()->command.position) > goal_position_threshold_)
      {
    	  if (now - goal_received_ < ros::Duration(1.0))
    	  {
    		  return;
    	  }
    	  else
    	  {
    		  ROS_ERROR("Cancelling goal: Controller is trying to achieve a different setpoint.");
    		  active_goal_.setCanceled();
    		  has_active_goal_ = false;
    	  }
      }


      control_msgs::GripperCommandFeedback feedback;
      feedback.position = msg->process_value;
      feedback.effort = msg->command;
      feedback.reached_goal = false;
      feedback.stalled = false;

      control_msgs::GripperCommandResult result;
      result.position = msg->process_value;
      result.effort = msg->command;
      result.reached_goal = false;
      result.stalled = false;

      std::cout << "process value: " << msg->process_value << std::endl;

      if (fabs(msg->process_value - active_goal_.getGoal()->command.position) < goal_position_threshold_)
      {
    	  feedback.reached_goal = true;

    	  result.reached_goal = true;
    	  active_goal_.setSucceeded(result);
    	  has_active_goal_ = false;
      }
      else
      {
    	  // Determines if the gripper has stalled.
    	  if (fabs(msg->process_value_dot) > stall_velocity_threshold_)
    	  {
    		  last_movement_time_ = ros::Time::now();
    	  }
    	  else if ((ros::Time::now() - last_movement_time_).toSec() > stall_timeout_ &&
               active_goal_.getGoal()->command.max_effort != 0.0)
    	  {
    		  feedback.stalled = true;

    		  result.stalled = true;
    		  active_goal_.setAborted(result);
    		  has_active_goal_ = false;
    	  }
      }
      active_goal_.publishFeedback(feedback);
  }
*/

}
