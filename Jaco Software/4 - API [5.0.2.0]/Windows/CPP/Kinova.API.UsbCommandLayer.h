#ifdef KINOVAAPIUSBCOMMANDLAYER_EXPORTS
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllexport)
#else
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllimport)
#endif

#include <vector>
#include "KinovaTypes.h"
#include <windows.h>

#define WIN32_LEAN_AND_MEAN

#define ERROR_INIT_API 2001
#define ERROR_LOAD_COMM_DLL 2002
#define JACO_NACK_FIRST 2003
#define JACO_COMM_FAILED 2004
#define JACO_NACK_NORMAL 2005

#define COMMAND_LAYER_VERSION 10001

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetJacoCount(DWORD &);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD InitAPI(void);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD CloseAPI(void);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCodeVersion(std::vector<int> &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCartesianPosition(CartesianPosition &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularPosition(AngularPosition &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCartesianForce(CartesianPosition &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularForce(AngularPosition &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularCurrent(AngularPosition &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetActualTrajectoryInfo(TrajectoryPoint &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetGlobalTrajectoryInfo(TrajectoryFIFO &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetSensorsInfo(SensorsInfo &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetSingularityVector(SingularityVector &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetAngularControl();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetCartesianControl();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD StartControlAPI();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD StopControlAPI();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD RestoreFactoryDefault();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendJoystickCommand(JoystickCommand joystickCommand);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendAdvanceTrajectory(TrajectoryPoint trajectory);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendBasicTrajectory(TrajectoryPoint trajectory);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetClientConfigurations(ClientConfigurations &config);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetClientConfigurations(ClientConfigurations config);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD EraseAllTrajectories();

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetPositionCurrentActuators(std::vector<float> &Response);

extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetActuatorPID(unsigned int address, float P, float I, float D);