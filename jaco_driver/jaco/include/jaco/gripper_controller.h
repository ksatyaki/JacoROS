/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * joint_trajectory_action_controller.h
 *
 *  Created on: 07.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef GRIPPER_CONTROLLER_H__
#define GRIPPER_CONTROLLER_H__


#include "ros/ros.h"

#include <actionlib/server/action_server.h>
#include <jaco/abstract_jaco.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>

namespace kinova
{

  class GripperAction
	{
	private:
	  typedef actionlib::ActionServer<control_msgs::GripperCommandAction> GAS;
	  typedef GAS::GoalHandle GoalHandle;
	public:
	  GripperAction(boost::shared_ptr<AbstractJaco> jaco);
	  ~GripperAction();

	private:

	  ros::NodeHandle node_;
	  boost::shared_ptr<AbstractJaco> jaco_;
	  GAS action_server_;
	  ros::Publisher pub_controller_command_;
	  ros::Subscriber sub_controller_state_;
	  ros::Timer watchdog_timer_;

	  bool has_active_goal_;
	  GoalHandle active_goal_;
	  ros::Time goal_received_;

	  double min_error_seen_;
	  double goal_threshold_;
	  double stall_velocity_threshold_;
	  double stall_timeout_;
	  ros::Time last_movement_time_;
	  control_msgs::JointControllerStateConstPtr last_controller_state_;

	  void watchdog(const ros::TimerEvent &e);

	  void goalCB(GoalHandle gh);

	  void cancelCB(GoalHandle gh);


	  void controlStateCB(const control_msgs::JointControllerStateConstPtr &msg);
	};

}

#endif /* GRIPPER_CONTROLLER_H__ */
