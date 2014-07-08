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
 *  Edited: Steffen Pfiffner / s.pfiffner@gmail.com
 *
 *
 *  FILE --- jaco_joystick_publisher.h
 *
 *  PURPOSE ---  Read the joystick values from jaco arm and publish the values as sensor_msgs/Joy
 *  			 see http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
 */

#ifndef JACO_JOYSTICK_PUBLISHER_H_
#define JACO_JOYSTICK_PUBLISHER_H_

#include <vector>
#include <jaco/abstract_jaco.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"


namespace kinova
{
	class JacoJoystickPublisher
	{
		public:
			JacoJoystickPublisher(boost::shared_ptr<AbstractJaco>);
			virtual ~JacoJoystickPublisher();
		  	void update();			
		private:
			boost::shared_ptr<AbstractJaco> jaco;
            ros::Publisher joystick_pub;

	};

}

#endif /* JACO_JOYSTICK_PUBLISHER_H_ */
