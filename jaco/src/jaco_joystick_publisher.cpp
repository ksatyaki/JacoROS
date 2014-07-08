/*
 * Copyright (c) 2011  DFKI GmbH, Bremen, Germany
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 *  Author: Sankaranarayanan Natarajan / sankar.natarajan@dfki.de
 *
 *  FILE --- jaco_joint_publisher.cpp
 *
 *  PURPOSE ---  Read the value from jaco arm and publish the values
 */

#include <jaco/jaco_joystick_publisher.h>

namespace kinova
{
	JacoJoystickPublisher::JacoJoystickPublisher(boost::shared_ptr<AbstractJaco> jaco) : jaco(jaco)
        {
                ros::NodeHandle nh;
                joystick_pub = nh.advertise<sensor_msgs::Joy>("jaco_joystick_state", 100);

        }

        JacoJoystickPublisher::~JacoJoystickPublisher()
        {
        }

	void JacoJoystickPublisher::update()
	{
		// publish joystick state
		sensor_msgs::JoyPtr joystick_msg = boost::make_shared<sensor_msgs::Joy>();

		joystick_msg->header.stamp = ros::Time::now();
		std::vector<float> float_vector(jaco->joystick_axes_states_.begin(), jaco->joystick_axes_states_.end());
		joystick_msg->axes = float_vector;

		std::vector<int> bool_vector(jaco->joystick_button_states_.begin(), jaco->joystick_button_states_.end());
		joystick_msg->buttons = bool_vector;

		joystick_pub.publish (joystick_msg);

	}
}
