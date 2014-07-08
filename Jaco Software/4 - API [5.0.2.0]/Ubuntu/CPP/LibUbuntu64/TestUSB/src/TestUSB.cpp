//============================================================================
// Name        : TestUSB.cpp
// Author      : Hugo
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <iostream>
#include <dlfcn.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

using namespace std;

#define RESET		0
#define BRIGHT 		1
#define DIM		    2
#define UNDERLINE 	3
#define BLINK		4
#define REVERSE		7
#define HIDDEN		8

#define BLACK 		0
#define RED		    1
#define GREEN		2
#define YELLOW		3
#define BLUE		4
#define MAGENTA		5
#define CYAN		6
#define	WHITE		7

#define FLAG_TEST_GETCODEVERSION true
#define GETCODEVERSION_TEST_COUNT 5000

#define FLAG_TEST_GETCARTESIANPOSITION true
#define GETCARTESIANPOSITION_TEST_COUNT 5000

#define FLAG_TEST_GETANGULARPOSITION true
#define GETANGULARPOSITION_TEST_COUNT 5000

#define FLAG_TEST_GETCARTESIANFORCE true
#define GETCARTESIANFORCE_TEST_COUNT 5000

#define FLAG_TEST_GETANGULARFORCE true
#define GETANGULARFORCE_TEST_COUNT 5000

#define FLAG_TEST_GETANGULARCURRENT true
#define GETANGULARCURRENT_TEST_COUNT 5000

#define FLAG_TEST_GETACTUALTRAJECTORYINFO true
#define GETACTUALTRAJECTORYINFO_TEST_COUNT 5000

#define FLAG_TEST_GETGLOBALTRAJECTORYINFO true
#define GETGLOBALTRAJECTORYINFO_TEST_COUNT 5000

#define FLAG_TEST_GETSENSORSINFO true
#define GETSENSORSINFO_TEST_COUNT 5000

#define FLAG_TEST_GETSINGULARITYVECTOR true
#define GETSINGULARITYVECTOR_TEST_COUNT 5000

#define FLAG_TEST_SETANGULARCONTROL true
#define SETANGULARCONTROL_TEST_COUNT 5000

#define FLAG_TEST_SETCARTESIANCONTROL true
#define SETCARTESIANCONTROL_TEST_COUNT 5000

#define FLAG_TEST_STARTCONTROLAPI true
#define STARTCONTROLAPI_TEST_COUNT 5000

#define FLAG_TEST_STOPCONTROLAPI true
#define STOPCONTROLAPI_TEST_COUNT 5000

#define FLAG_TEST_RESTOREFACTORYDEFAULT true
#define RESTOREFACTORYDEFAULT_TEST_COUNT 5000

#define FLAG_TEST_SENDJOYSTICKCOMMAND false
#define SENDJOYSTICKCOMMAND_TEST_COUNT 5000

#define FLAG_TEST_GETANGULARPOSITION true
#define GETANGULARPOSITION_TEST_COUNT 5000

#define FLAG_TEST_SENDADVANCETRAJECTORY false
#define SENDADVANCETRAJECTORY_TEST_COUNT 5000

#define FLAG_TEST_SENDBASICTRAJECTORY false
#define SENDBASICTRAJECTORY_TEST_COUNT 5000

#define FLAG_TEST_GETCLIENTCONFIG true
#define GETCLIENTCONFIG_TEST_COUNT 50

#define FLAG_TEST_SETCLIENTCONFIG true
#define SETCLIENTCONFIG_TEST_COUNT 50

#define FLAG_TEST_ERASEALLTRAJECTORY true
#define ERASEALLTRAJECTORY_TEST_COUNT 50

#define FLAG_TEST_SETACTUATORPID false
#define SETACTUATORPID_TEST_COUNT 5000

#define FLAG_TEST_GETPOSITIONCURRENTACTUATORS true
#define GETPOSITIONCURRENTACTUATORS_TEST_COUNT 5000

void textcolor(int attr, int fg, int bg);
void DisplayLoadResult(char* methodName, bool result);
void DisplayClientConfigurations(ClientConfigurations config);
void DisplayCartesianPosition(CartesianPosition position);
void DisplaySingularityVector(SingularityVector vector);
void DisplaySensorsInfo(SensorsInfo info);
void DisplayTrajectoryFIFO(TrajectoryFIFO fifo);
void DisplayTrajectoryPoint(TrajectoryPoint trajPoint);
void DisplayAngularPosition(AngularPosition position);
void DisplayTestResult(char* testName, bool result);

bool TestGetCodeVersion();
bool TestGetCartesianPosition();
bool TestGetAngularPosition();
bool TestGetCartesianForce();
bool TestGetAngularForce();
bool TestGetAngularCurrent();
bool TestGetActualTrajectoryInfo();
bool TestGetGlobalTrajectoryInfo();
bool TestGetSensorsInfo();
bool TestGetSingularityVector();
bool TestSetAngularControl();
bool TestSetCartesianControl();
bool TestStartControlAPI();
bool TestStopControlAPI();
bool TestRestoreFactoryDefault();
bool TestSendJoystickCommand();
bool TestSendAdvanceTrajectory();
bool TestSendBasicTrajectory();
bool TestGetClientConfigurations();
bool TestSetClientConfig();
bool TestEraseAllTrajectory();
bool TestGetPositionCurrentActuators();
bool TestSetActuatorPID();


int (*MyGetClientConfigurations)(ClientConfigurations &);
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MyGetCodeVersion)(std::vector<int> &);
int (*MyGetCartesianPosition)(CartesianPosition &);
int (*MyGetAngularPosition)(AngularPosition &);
int (*MyGetCartesianForce)(CartesianPosition &);
int (*MyGetAngularForce)(AngularPosition &);
int (*MyGetAngularCurrent)(AngularPosition &);
int (*MyGetActualTrajectoryInfo)(TrajectoryPoint &);
int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &);
int (*MyGetSensorsInfo)(SensorsInfo &);
int (*MyGetSingularityVector)(SingularityVector &);
int (*MySetAngularControl)();
int (*MySetCartesianControl)();
int (*MyStartControlAPI)();
int (*MyStopControlAPI)();
int (*MyRestoreFactoryDefault)();
int (*MySendJoystickCommand)(JoystickCommand joystickCOmmand);
int (*MySendAdvanceTrajectory)(TrajectoryPoint trajectory);
int (*MySendBasicTrajectory)(TrajectoryPoint trajectory);
int (*MySetClientConfigurations)(ClientConfigurations);
int (*MyEraseAllTrajectory)();
int (*MyGetPositionCurrentActuators)(std::vector<float> &Response);
int (*MySetActuatorPID)(unsigned int address, float P, float I, float D);

void InitMethod();

void textcolor(int attr, int fg, int bg)
{
	char command[13];

	/* Command is the control command to the terminal */
	sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
	printf("%s", command);
}
void DisplayLoadResult(char* methodName, bool result)
{
	textcolor(BRIGHT, WHITE, BLACK);
	printf("Loading %s method -----> [", methodName);
	if(result == false)
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("FAILED");
		textcolor(BRIGHT, WHITE, BLACK);
		printf("]\n");
	}
	else
	{
		textcolor(BRIGHT, GREEN, BLACK);
		printf("SUCCESS");
		textcolor(BRIGHT, WHITE, BLACK);
		printf("]\n");
	}
}
void DisplayClientConfigurations(ClientConfigurations config)
{
	printf("Client id = %s\n", config.ClientID);
	printf("Client ClientName = %s\n", config.ClientName);
	printf("Client ComplexRetractActive = %d\n", config.ComplexRetractActive);
	printf("Client DeletePreProgrammedPositionsAtRetract = %d\n", config.DeletePreProgrammedPositionsAtRetract);
	printf("Client DrinkingDistance = %f\n", config.DrinkingDistance);
	printf("Client DrinkingHeight = %f\n", config.DrinkingHeight);
	printf("Client DrinkingLenght = %f\n", config.DrinkingLenght);
	printf("Client EnableFlashErrorLog = %d\n", config.EnableFlashErrorLog);
	printf("Client EnableFlashPositionLog = %d\n", config.EnableFlashPositionLog);
	printf("Client Fingers2and3Inverted = %d\n", config.Fingers2and3Inverted);
	printf("Client Laterality = %d\n", config.Laterality);
	printf("Client MaxAngularAcceleration = %f\n", config.MaxAngularAcceleration);
	printf("Client MaxAngularSpeed = %f\n", config.MaxAngularSpeed);
	printf("Client MaxForce = %f\n", config.MaxForce);
	printf("Client MaxLinearAcceleration = %f\n", config.MaxLinearAcceleration);
	printf("Client MaxLinearSpeed = %f\n", config.MaxLinearSpeed);
	printf("Client Model = %s\n", config.Model);
	printf("Client Organization = %s\n", config.Organization);
	printf("Client RetractedPositionAngle = %f\n", config.RetractedPositionAngle);
	printf("Client RetractedPositionCount = %d\n", config.RetractedPositionCount);
	printf("Client Sensibility = %f\n", config.Sensibility);
	printf("Client Serial = %s\n", config.Serial);

	for(int i = 0; i < config.RetractedPositionCount; i++)
	{
		printf("	Retracted position %d Actuator 1 = %f\n",i , config.RetractPositions[i].Actuators.Actuator1);
		printf("	Retracted position %d Actuator 2 = %f\n",i , config.RetractPositions[i].Actuators.Actuator2);
		printf("	Retracted position %d Actuator 3 = %f\n",i , config.RetractPositions[i].Actuators.Actuator3);
		printf("	Retracted position %d Actuator 4 = %f\n",i , config.RetractPositions[i].Actuators.Actuator4);
		printf("	Retracted position %d Actuator 5 = %f\n",i , config.RetractPositions[i].Actuators.Actuator5);
		printf("	Retracted position %d Actuator 6 = %f\n",i , config.RetractPositions[i].Actuators.Actuator6);

		printf("	Retracted position %d X = %f\n",i , config.RetractPositions[i].CartesianPosition.X);
		printf("	Retracted position %d Y = %f\n",i , config.RetractPositions[i].CartesianPosition.Y);
		printf("	Retracted position %d Z = %f\n",i , config.RetractPositions[i].CartesianPosition.Z);
		printf("	Retracted position %d ThetaX = %f\n",i , config.RetractPositions[i].CartesianPosition.ThetaX);
		printf("	Retracted position %d ThetaY = %f\n",i , config.RetractPositions[i].CartesianPosition.ThetaY);
		printf("	Retracted position %d ThetaZ = %f\n",i , config.RetractPositions[i].CartesianPosition.ThetaZ);

		printf("	Retracted position %d delay = %f\n",i , config.RetractPositions[i].Delay);
		printf("	Retracted position %d Finger 1 = %f\n",i , config.RetractPositions[i].Fingers.Finger1);
		printf("	Retracted position %d Finger 2 = %f\n",i , config.RetractPositions[i].Fingers.Finger2);
		printf("	Retracted position %d Finger 3 = %f\n",i , config.RetractPositions[i].Fingers.Finger3);
		printf("	Retracted position %d hand mode = %d\n",i , config.RetractPositions[i].HandMode);
		printf("	Retracted position %d type = %d\n",i , config.RetractPositions[i].Type);
	}
}
void DisplayCartesianPosition(CartesianPosition position)
{
	printf("X = %f\n", position.Coordinates.X);
	printf("Y = %f\n", position.Coordinates.Y);
	printf("Z = %f\n", position.Coordinates.Z);
	printf("ThetaX = %f\n", position.Coordinates.ThetaX);
	printf("ThetaY = %f\n", position.Coordinates.ThetaY);
	printf("ThetaZ = %f\n", position.Coordinates.ThetaZ);
	printf("Finger 1 = %f\n", position.Fingers.Finger1);
	printf("Finger 2 = %f\n", position.Fingers.Finger2);
	printf("Finger 3 = %f\n", position.Fingers.Finger3);
}
void DisplaySingularityVector(SingularityVector vector)
{
	printf("OrientationSingularityCount = %d\n", vector.OrientationSingularityCount);
	printf("OrientationSingularityDistance = %f\n", vector.OrientationSingularityDistance);
	printf("RepulsionVector.X = %f\n", vector.RepulsionVector.X);
	printf("RepulsionVector.Y = %f\n", vector.RepulsionVector.Y);
	printf("RepulsionVector.Z = %f\n", vector.RepulsionVector.Z);
	printf("RepulsionVector.ThetaX = %f\n", vector.RepulsionVector.ThetaX);
	printf("RepulsionVector.ThetaY = %f\n", vector.RepulsionVector.ThetaY);
	printf("RepulsionVector.ThetaZ = %f\n", vector.RepulsionVector.ThetaZ);
	printf("TranslationSingularityCount = %d\n", vector.TranslationSingularityCount);
	printf("TranslationSingularityDistance = %f\n", vector.TranslationSingularityDistance);
}
void DisplaySensorsInfo(SensorsInfo info)
{
	printf("AccelerationX = %f\n", info.AccelerationX);
	printf("AccelerationY = %f\n", info.AccelerationY);
	printf("AccelerationZ = %f\n", info.AccelerationZ);
	printf("ActuatorTemp1 = %f\n", info.ActuatorTemp1);
	printf("ActuatorTemp2 = %f\n", info.ActuatorTemp2);
	printf("ActuatorTemp3 = %f\n", info.ActuatorTemp3);
	printf("ActuatorTemp4 = %f\n", info.ActuatorTemp4);
	printf("ActuatorTemp5 = %f\n", info.ActuatorTemp5);
	printf("ActuatorTemp6 = %f\n", info.ActuatorTemp6);
	printf("Current = %f\n", info.Current);
	printf("FingerTemp1 = %f\n", info.FingerTemp1);
	printf("FingerTemp2 = %f\n", info.FingerTemp2);
	printf("FingerTemp3 = %f\n", info.FingerTemp3);
	printf("Voltage = %f\n", info.Voltage);
}
void DisplayTrajectoryFIFO(TrajectoryFIFO fifo)
{
	printf("Max size = %d\n", fifo.MaxSize);
	printf("Trajectory count = %d\n", fifo.TrajectoryCount);
	printf("Used percentage = %f\n", fifo.UsedPercentage);
}
void DisplayTrajectoryPoint(TrajectoryPoint trajPoint)
{
	printf("Limitation - SpeedParameter1 = %f\n", trajPoint.Limitations.speedParameter1);
	printf("Limitation - SpeedParameter2 = %f\n", trajPoint.Limitations.speedParameter2);
	printf("Limitation - SpeedParameter3 = %f\n", trajPoint.Limitations.speedParameter3);
	printf("Limitation - accelerationParameter1 = %f\n", trajPoint.Limitations.accelerationParameter1);
	printf("Limitation - accelerationParameter2 = %f\n", trajPoint.Limitations.accelerationParameter2);
	printf("Limitation - accelerationParameter3 = %f\n", trajPoint.Limitations.accelerationParameter3);
	printf("Limitation - forceParameter1 = %f\n", trajPoint.Limitations.forceParameter1);
	printf("Limitation - forceParameter2 = %f\n", trajPoint.Limitations.forceParameter2);
	printf("Limitation - forceParameter3 = %f\n", trajPoint.Limitations.forceParameter3);
	printf("Limitation active = %d\n", trajPoint.LimitationsActive);
	printf("Position - Actuator 1 = %f\n", trajPoint.Position.Actuators.Actuator1);
	printf("Position - Actuator 2 = %f\n", trajPoint.Position.Actuators.Actuator2);
	printf("Position - Actuator 3 = %f\n", trajPoint.Position.Actuators.Actuator3);
	printf("Position - Actuator 4 = %f\n", trajPoint.Position.Actuators.Actuator4);
	printf("Position - Actuator 5 = %f\n", trajPoint.Position.Actuators.Actuator5);
	printf("Position - Actuator 6 = %f\n", trajPoint.Position.Actuators.Actuator6);
	printf("Position - X = %f\n", trajPoint.Position.CartesianPosition.X);
	printf("Position - Y = %f\n", trajPoint.Position.CartesianPosition.Y);
	printf("Position - Z = %f\n", trajPoint.Position.CartesianPosition.Z);
	printf("Position - ThetaX = %f\n", trajPoint.Position.CartesianPosition.ThetaX);
	printf("Position - ThetaY = %f\n", trajPoint.Position.CartesianPosition.ThetaY);
	printf("Position - ThetaZ = %f\n", trajPoint.Position.CartesianPosition.ThetaZ);
	printf("Position - Delay = %f\n", trajPoint.Position.Delay);
	printf("Position - Finger 1 = %f\n", trajPoint.Position.Fingers.Finger1);
	printf("Position - Finger 2 = %f\n", trajPoint.Position.Fingers.Finger2);
	printf("Position - Finger 3 = %f\n", trajPoint.Position.Fingers.Finger3);
	printf("Position - HandMode = %d\n", trajPoint.Position.HandMode);
	printf("Position - Type = %d\n", trajPoint.Position.Type);
}
void DisplayAngularPosition(AngularPosition position)
{
	printf("Actuator 1 = %f\n", position.Actuators.Actuator1);
	printf("Actuator 2 = %f\n", position.Actuators.Actuator2);
	printf("Actuator 3 = %f\n", position.Actuators.Actuator3);
	printf("Actuator 4 = %f\n", position.Actuators.Actuator4);
	printf("Actuator 5 = %f\n", position.Actuators.Actuator5);
	printf("Actuator 6 = %f\n", position.Actuators.Actuator6);
	printf("Finger 1 = %f\n", position.Fingers.Finger1);
	printf("Finger 2 = %f\n", position.Fingers.Finger2);
	printf("Finger 3 = %f\n", position.Fingers.Finger3);
}
void DisplayTestResult(char* testName, bool result)
{
	textcolor(BRIGHT, WHITE, BLACK);
	printf("Testing %s method -----> [", testName);
	if(result == false)
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("FAILED");
		textcolor(BRIGHT, WHITE, BLACK);
		printf("]\n");
	}
	else
	{
		textcolor(BRIGHT, GREEN, BLACK);
		printf("SUCCESS");
		textcolor(BRIGHT, WHITE, BLACK);
		printf("]\n");
	}
}

bool TestGetPositionCurrentActuators()
{
	bool result = true;
	std::vector<float> data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETPOSITIONCURRENTACTUATORS_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetPositionCurrentActuators)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetPositionCurrentActuators", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}

bool TestSetActuatorPID()
{
	bool result = true;
	std::vector<int> data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SETACTUATORPID_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySetActuatorPID)(16, 0.5f, 0.0f, 0.0f);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SetActuatorPID", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}

bool TestGetCodeVersion()
{
	bool result = true;
	std::vector<int> data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETCODEVERSION_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetCodeVersion)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetCodeVersion", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetCartesianPosition()
{
	bool result = true;
	CartesianPosition data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETCARTESIANPOSITION_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetCartesianPosition)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetCartesianPosition", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetAngularPosition()
{
	bool result = true;
	AngularPosition data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETANGULARPOSITION_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetAngularPosition)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetAngularPosition", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetCartesianForce()
{
	bool result = true;
	CartesianPosition data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETCARTESIANFORCE_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetCartesianForce)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetCartesianForce", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetAngularForce()
{
	bool result = true;
	AngularPosition data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETANGULARFORCE_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetAngularForce)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetAngularForce", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetAngularCurrent()
{
	bool result = true;
	AngularPosition data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETANGULARCURRENT_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetAngularCurrent)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetAngularCurrent", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetActualTrajectoryInfo()
{
	bool result = true;
	TrajectoryPoint data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETACTUALTRAJECTORYINFO_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetActualTrajectoryInfo)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetActualTrajectoryInfo", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetGlobalTrajectoryInfo()
{
	bool result = true;
	TrajectoryFIFO data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETGLOBALTRAJECTORYINFO_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetGlobalTrajectoryInfo)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetActualTrajectoryInfo", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetSensorsInfo()
{
	bool result = true;
	SensorsInfo data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETSENSORSINFO_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetSensorsInfo)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetSensorsInfo", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetSingularityVector()
{
	bool result = true;
	SingularityVector data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETSINGULARITYVECTOR_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetSingularityVector)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetSingularityVector", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSetAngularControl()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SETANGULARCONTROL_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySetAngularControl)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SetAngularControl", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSetCartesianControl()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SETCARTESIANCONTROL_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySetCartesianControl)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SetCartesianControl", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestStartControlAPI()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = STARTCONTROLAPI_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyStartControlAPI)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("StartControlAPI", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestStopControlAPI()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = STOPCONTROLAPI_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyStopControlAPI)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("StopControlAPI", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestRestoreFactoryDefault()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = RESTOREFACTORYDEFAULT_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyRestoreFactoryDefault)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("RestoreFactoryDefault", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSendJoystickCommand()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SENDJOYSTICKCOMMAND_TEST_COUNT;

	JoystickCommand data;

	for(int i = 0; i < 16; i++)
	{
		data.ButtonValue[i] = 0;
	}
	data.InclineForwardBackward = 0.0f;
	data.InclineLeftRight = 0.0f;
	data.MoveForwardBackward = 0.0f;
	data.MoveLeftRight = 0.0f;
	data.PushPull = 0.0f;
	data.Rotate = 0.0f;

	(*MyStartControlAPI)();

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySendJoystickCommand)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	(*MyStopControlAPI)();

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SendJoystickCommand", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSendAdvanceTrajectory()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SENDADVANCETRAJECTORY_TEST_COUNT;
	TrajectoryPoint data;

	data.Limitations.accelerationParameter1 = 0.0f;
	data.Limitations.accelerationParameter2 = 0.0f;
	data.Limitations.accelerationParameter3 = 0.0f;
	data.Limitations.forceParameter1 = 0.0f;
	data.Limitations.forceParameter2 = 0.0f;
	data.Limitations.forceParameter3 = 0.0f;
	data.Limitations.speedParameter1 = 0.0f;
	data.Limitations.speedParameter2 = 0.0f;
	data.Limitations.speedParameter3 = 0.0f;

	data.LimitationsActive = false;

	data.Position.Actuators.Actuator1 = 0.0f;
	data.Position.Actuators.Actuator2 = 0.0f;
	data.Position.Actuators.Actuator3 = 0.0f;
	data.Position.Actuators.Actuator4 = 0.0f;
	data.Position.Actuators.Actuator5 = 0.0f;
	data.Position.Actuators.Actuator6 = 0.0f;

	data.Position.CartesianPosition.X = 0.0f;
	data.Position.CartesianPosition.Y = 0.0f;
	data.Position.CartesianPosition.Z = 0.0f;
	data.Position.CartesianPosition.ThetaX = 0.0f;
	data.Position.CartesianPosition.ThetaY = 0.0f;
	data.Position.CartesianPosition.ThetaZ = 0.0f;

	data.Position.Delay = 0.0f;

	data.Position.Fingers.Finger1 = 0.0f;
	data.Position.Fingers.Finger2 = 0.0f;
	data.Position.Fingers.Finger3 = 0.0f;

	data.Position.HandMode = THREEFINGER;
	data.Position.Type = CARTESIAN_POSITION;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySendAdvanceTrajectory)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SendAdvanceTrajectory", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSendBasicTrajectory()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SENDBASICTRAJECTORY_TEST_COUNT;
	TrajectoryPoint data;

	data.Limitations.accelerationParameter1 = 0.0f;
	data.Limitations.accelerationParameter2 = 0.0f;
	data.Limitations.accelerationParameter3 = 0.0f;
	data.Limitations.forceParameter1 = 0.0f;
	data.Limitations.forceParameter2 = 0.0f;
	data.Limitations.forceParameter3 = 0.0f;
	data.Limitations.speedParameter1 = 0.0f;
	data.Limitations.speedParameter2 = 0.0f;
	data.Limitations.speedParameter3 = 0.0f;

	data.LimitationsActive = false;

	data.Position.Actuators.Actuator1 = 0.0f;
	data.Position.Actuators.Actuator2 = 0.0f;
	data.Position.Actuators.Actuator3 = 0.0f;
	data.Position.Actuators.Actuator4 = 0.0f;
	data.Position.Actuators.Actuator5 = 0.0f;
	data.Position.Actuators.Actuator6 = 0.0f;

	data.Position.CartesianPosition.X = 0.0f;
	data.Position.CartesianPosition.Y = 0.0f;
	data.Position.CartesianPosition.Z = 0.0f;
	data.Position.CartesianPosition.ThetaX = 0.0f;
	data.Position.CartesianPosition.ThetaY = 0.0f;
	data.Position.CartesianPosition.ThetaZ = 0.0f;

	data.Position.Delay = 0.0f;

	data.Position.Fingers.Finger1 = 0.0f;
	data.Position.Fingers.Finger2 = 0.0f;
	data.Position.Fingers.Finger3 = 0.0f;

	data.Position.HandMode = THREEFINGER;
	data.Position.Type = CARTESIAN_POSITION;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySendBasicTrajectory)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SendBasicTrajectory", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestGetClientConfigurations()
{
	bool result = true;
	ClientConfigurations data;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = GETCLIENTCONFIG_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyGetClientConfigurations)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;
	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("GetClientConfigurations", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestSetClientConfig()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = SETCLIENTCONFIG_TEST_COUNT;
	ClientConfigurations data;

	(*MyGetClientConfigurations)(data);

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MySetClientConfigurations)(data);

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("SetClientConfigurations", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}
bool TestEraseAllTrajectory()
{
	bool result = true;

	struct timespec t2, t3;
	double dt1;
	double rate;
	int operationCount = ERASEALLTRAJECTORY_TEST_COUNT;

	clock_gettime(CLOCK_MONOTONIC,  &t2);

	for(int i = 0; i < operationCount; i++)
	{
		result = (*MyEraseAllTrajectory)();

		if(result != 1)
		{
			result = false;
		}
	}

	clock_gettime(CLOCK_MONOTONIC,  &t3);

	dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) / 1e9;

	rate = (double)operationCount /  dt1;

	printf("\n\n");
	DisplayTestResult("EraseAllTrajectory", result);
	textcolor(BRIGHT, MAGENTA, BLACK);
	printf("	Operation count = %d\n", operationCount);
	printf("	Time = %f seconds\n", dt1);
	printf("	Rate = %f Hz\n", rate);

	textcolor(BRIGHT, WHITE, BLACK);
	return result;
}


void * commLayer_handle;
void * commandLayer_handle;

ClientConfigurations clientConfig;
CartesianPosition cartesianPosition;
JoystickCommand joystickCommand;
SingularityVector singularityVector;
SensorsInfo sensorsInfo;
TrajectoryFIFO trajectoryFIFO;
AngularPosition angularPosition;
std::vector<int> vectorCodeVersion;

int main()
{
	textcolor(BRIGHT, BLUE, BLACK);
	printf("\n\n- - - WELCOME TO C++ API TEST PROGRAM - - - \n\n\n");
	textcolor(BRIGHT, WHITE, BLACK);

	InitMethod();

	int result = 0;

	result = (*MyInitAPI)();
	printf("Result of InitAPI = %d\n", result);

	result = (*MyRestoreFactoryDefault)();
	printf("RestoreFactoryDefault's result = %d\n", result);

	result = (*MyGetClientConfigurations)(clientConfig);
	//DisplayClientConfigurations(clientConfig);

	memcpy(clientConfig.ClientID, "yoyo", 20);
	clientConfig.MaxLinearSpeed = 0.13f;
	result = (*MySetClientConfigurations)(clientConfig);

	result = (*MyGetClientConfigurations)(clientConfig);
	printf("GetClientConfigurations's result = %d\n", result);
	//DisplayClientConfigurations(clientConfig);

	TestGetCodeVersion();
	TestGetCartesianPosition();
	TestGetAngularPosition();
	TestGetCartesianForce();
	TestGetAngularForce();
	TestGetAngularCurrent();
	TestGetActualTrajectoryInfo();
	TestGetGlobalTrajectoryInfo();
	TestGetSensorsInfo();
	TestGetSingularityVector();
	TestSetAngularControl();
	TestSetCartesianControl();
	TestStartControlAPI();
	TestStopControlAPI();
	TestRestoreFactoryDefault();
	TestSendJoystickCommand();


	TestSendAdvanceTrajectory();

	TestSendBasicTrajectory();


	TestGetClientConfigurations();
	TestSetClientConfig();
	TestEraseAllTrajectory();

	TestGetPositionCurrentActuators();
	TestSetActuatorPID();

	//(*MySetActuatorPID)(16, 0.5f, 0.0f, 0.0f);

	std::vector<float> Response;

	(*MyGetPositionCurrentActuators)(Response);

	printf("Position Joint1 = %f\n", Response[0]);
	printf("Position Joint2 = %f\n", Response[1]);
	printf("Position Joint3 = %f\n", Response[2]);
	printf("Position Joint4 = %f\n", Response[3]);
	printf("Position Joint5 = %f\n", Response[4]);
	printf("Position Joint6 = %f\n", Response[5]);
	printf("Current Joint1 = %f\n", Response[6]);
	printf("Current Joint2 = %f\n", Response[7]);
	printf("Current Joint3 = %f\n", Response[8]);
	printf("Current Joint4 = %f\n", Response[9]);
	printf("Current Joint5 = %f\n", Response[10]);
	printf("Current Joint6 = %f\n", Response[11]);

	(*MyCloseAPI)();

	printf("End of test.\n");
	return 0;
}

void InitMethod()
{
	commandLayer_handle = dlopen("./Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	if(commandLayer_handle==NULL)
	{
		textcolor(BRIGHT, RED, BLACK);
		printf("Error while loading library. Cannot perform test. Leaving,,,\n");
	}
	else
	{
		textcolor(BRIGHT, GREEN, BLACK);
		printf("API loaded Successfully\n");
	}

	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	if(MyInitAPI==NULL)
	{
		DisplayLoadResult("InitAPI", false);
	}
	else
	{
		DisplayLoadResult("InitAPI", true);
	}

	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	if(MyCloseAPI==NULL)
	{
		DisplayLoadResult("CloseAPI", false);
	}
	else
	{
		DisplayLoadResult("CloseAPI", true);
	}

	MyRestoreFactoryDefault = (int (*)()) dlsym(commandLayer_handle,"RestoreFactoryDefault");
	if(RestoreFactoryDefault==NULL)
	{
		DisplayLoadResult("RestoreFactoryDefault", false);
	}
	else
	{
		DisplayLoadResult("RestoreFactoryDefault", true);
	}

	MyEraseAllTrajectory = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
	if(MyEraseAllTrajectory==NULL)
	{
		DisplayLoadResult("EraseAllTrajectory", false);
	}
	else
	{
		DisplayLoadResult("EraseAllTrajectory", true);
	}

	MyGetClientConfigurations = (int (*)(ClientConfigurations &)) dlsym(commandLayer_handle,"GetClientConfigurations");
	if(GetClientConfigurations==NULL)
	{
		DisplayLoadResult("GetClientConfigurations", false);
	}
	else
	{
		DisplayLoadResult("GetClientConfigurations", true);
	}

	if((MySetClientConfigurations = (int (*)(ClientConfigurations)) dlsym(commandLayer_handle,"SetClientConfigurations")) == NULL)
	{
		DisplayLoadResult("SetClientConfigurations", false);
	}
	else
	{
		DisplayLoadResult("SetClientConfigurations", true);
	}

	if((MyStartControlAPI = (int (*)()) dlsym(commandLayer_handle,"StartControlAPI")) == NULL)
	{
		DisplayLoadResult("StartControlAPI", false);
	}
	else
	{
		DisplayLoadResult("StartControlAPI", true);
	}

	if((MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition")) == NULL)
	{
		DisplayLoadResult("GetCartesianPosition", false);
	}
	else
	{
		DisplayLoadResult("GetCartesianPosition", true);
	}

	if((MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory")) == NULL)
	{
		DisplayLoadResult("SendAdvanceTrajectory", false);
	}
	else
	{
		DisplayLoadResult("SendAdvanceTrajectory", true);
	}

	if((MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory")) == NULL)
	{
		DisplayLoadResult("SendBasicTrajectory", false);
	}
	else
	{
		DisplayLoadResult("SendBasicTrajectory", true);
	}

	if((MySendJoystickCommand = (int (*)(JoystickCommand)) dlsym(commandLayer_handle,"SendJoystickCommand")) == NULL)
	{
		DisplayLoadResult("SendJoystickCommand", false);
	}
	else
	{
		DisplayLoadResult("SendJoystickCommand", true);
	}

	if((MyStopControlAPI = (int (*)()) dlsym(commandLayer_handle,"StopControlAPI")) == NULL)
	{
		DisplayLoadResult("StopControlAPI", false);
	}
	else
	{
		DisplayLoadResult("StopControlAPI", true);
	}

	if((MySetCartesianControl = (int (*)()) dlsym(commandLayer_handle,"SetCartesianControl")) == NULL)
	{
		DisplayLoadResult("SetCartesianControl", false);
	}
	else
	{
		DisplayLoadResult("SetCartesianControl", true);
	}


	if((MySetAngularControl = (int (*)()) dlsym(commandLayer_handle,"SetAngularControl")) == NULL)
	{
		DisplayLoadResult("SetAngularControl", false);
	}
	else
	{
		DisplayLoadResult("SetAngularControl", true);
	}

	if((MyGetSingularityVector = (int (*)(SingularityVector &)) dlsym(commandLayer_handle,"GetSingularityVector")) == NULL)
	{
		DisplayLoadResult("GetSingularityVector", false);
	}
	else
	{
		DisplayLoadResult("GetSingularityVector", true);
	}

	if((MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle,"GetSensorsInfo")) == NULL)
	{
		DisplayLoadResult("GetSensorsInfo", false);
	}
	else
	{
		DisplayLoadResult("GetSensorsInfo", true);
	}

	if((MyGetGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &)) dlsym(commandLayer_handle,"GetGlobalTrajectoryInfo")) == NULL)
	{
		DisplayLoadResult("GetGlobalTrajectoryInfo", false);
	}
	else
	{
		DisplayLoadResult("GetGlobalTrajectoryInfo", true);
	}

	if((MyGetActualTrajectoryInfo = (int (*)(TrajectoryPoint &)) dlsym(commandLayer_handle,"GetActualTrajectoryInfo")) == NULL)
	{
		DisplayLoadResult("GetActualTrajectoryInfo", false);
	}
	else
	{
		DisplayLoadResult("GetActualTrajectoryInfo", true);
	}

	if((MyGetAngularCurrent = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCurrent")) == NULL)
	{
		DisplayLoadResult("GetAngularCurrent", false);
	}
	else
	{
		DisplayLoadResult("GetAngularCurrent", true);
	}

	if((MyGetAngularForce = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularForce")) == NULL)
	{
		DisplayLoadResult("GetAngularForce", false);
	}
	else
	{
		DisplayLoadResult("GetAngularForce", true);
	}

	if((MyGetCartesianForce = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianForce")) == NULL)
	{
		DisplayLoadResult("GetCartesianForce", false);
	}
	else
	{
		DisplayLoadResult("GetCartesianForce", true);
	}

	if((MyGetAngularPosition = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularPosition")) == NULL)
	{
		DisplayLoadResult("GetAngularPosition", false);
	}
	else
	{
		DisplayLoadResult("GetAngularPosition", true);
	}

	if((MyGetCartesianPosition = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianPosition")) == NULL)
	{
		DisplayLoadResult("GetCartesianPosition", false);
	}
	else
	{
		DisplayLoadResult("GetCartesianPosition", true);
	}

	if((MyGetCodeVersion = (int (*)(std::vector<int> &)) dlsym(commandLayer_handle,"GetCodeVersion")) == NULL)
	{
		DisplayLoadResult("GetCodeVersion", false);
	}
	else
	{
		DisplayLoadResult("GetCodeVersion", true);
	}

	if((MySetActuatorPID = (int (*)(unsigned int address, float P, float I, float D)) dlsym(commandLayer_handle,"SetActuatorPID")) == NULL)
	{
		DisplayLoadResult("SetActuatorPID", false);
	}
	else
	{
		DisplayLoadResult("SetActuatorPID", true);
	}

	if((MyGetPositionCurrentActuators = (int (*)(std::vector<float> &)) dlsym(commandLayer_handle,"GetPositionCurrentActuators")) == NULL)
	{
		DisplayLoadResult("GetPositionCurrentActuators", false);
	}
	else
	{
		DisplayLoadResult("GetPositionCurrentActuators", true);
	}
}

