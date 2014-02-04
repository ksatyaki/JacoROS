// TestCommandLayer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Kinova.API.UsbCommandLayer.h"
#include "Kinova.DLL.CommLayer.h"
#include "mpusbapi.h"
#include <windows.h>
#include <conio.h>
#include <vector>
#include "KinovaTypes.h"

int _tmain(int argc, _TCHAR* argv[])
{
	//A variable to store the result of our operations.
	int result = NO_ERROR;

	//A handle to the API.
	HINSTANCE handleAPI;
	
    handleAPI = NULL;

	//We load the API.
    handleAPI = LoadLibrary(L"Kinova.API.UsbCommandLayer.dll");
    
	if(handleAPI == NULL)
    {
        printf("Error while loading Command");
    }
    else
    {
		//A function pointer for each function of the API we need to use.
		DWORD (*InitAPI)();
		
		//We load the function into the pointer.
		InitAPI=(DWORD(*)()) GetProcAddress(handleAPI,"InitAPI");

		DWORD (*CloseAPI)();
		CloseAPI=(DWORD(*)()) GetProcAddress(handleAPI,"CloseAPI");

		DWORD (*GetClientConfigurations)(ClientConfigurations &);
		GetClientConfigurations=(DWORD(*)(ClientConfigurations &)) GetProcAddress(handleAPI,"GetClientConfigurations");

		DWORD (*SetClientConfigurations)(ClientConfigurations);
		SetClientConfigurations=(DWORD(*)(ClientConfigurations)) GetProcAddress(handleAPI,"SetClientConfigurations");

		DWORD (*GetCartesianPosition)(CartesianPosition &);
		GetCartesianPosition=(DWORD(*)(CartesianPosition&))GetProcAddress(handleAPI,"GetCartesianPosition");

		DWORD (*GetJacoCount)(DWORD &);
		GetJacoCount=(DWORD(*)(DWORD&))GetProcAddress(handleAPI,"GetJacoCount");

		ClientConfigurations config;
		std::vector<float> PositionCurrent;

		DWORD (*SendAdvanceTrajectory)(TrajectoryPoint);
		SendAdvanceTrajectory=(DWORD(*)(TrajectoryPoint)) GetProcAddress(handleAPI,"SendAdvanceTrajectory");

		DWORD (*StartControlAPI)();
		StartControlAPI=(DWORD(*)()) GetProcAddress(handleAPI,"StartControlAPI");

		DWORD (*GetPositionCurrentActuators)(std::vector<float> &);
		GetPositionCurrentActuators=(DWORD(*)(std::vector<float> &)) GetProcAddress(handleAPI,"GetPositionCurrentActuators");

		DWORD (*SetActuatorPID)(unsigned int address, float P, float I, float D);
		SetActuatorPID=(DWORD(*)(unsigned int, float, float, float)) GetProcAddress(handleAPI,"SetActuatorPID");

		//We call the function InitAPI.
		result = (*InitAPI)();

		DWORD temp;

		if(result == 1)
		{
			printf("Get jaco count = %d\n", (*GetJacoCount)(temp));
			(*GetClientConfigurations)(config);
			printf("speed = %f\n", config.MaxLinearSpeed);
			(*GetPositionCurrentActuators)(PositionCurrent);

			printf("Position Joint 1 = %f\n", PositionCurrent[0]);
			printf("Position Joint 2 = %f\n", PositionCurrent[1]);
			printf("Position Joint 3 = %f\n", PositionCurrent[2]);
			printf("Position Joint 4 = %f\n", PositionCurrent[3]);
			printf("Position Joint 5 = %f\n", PositionCurrent[4]);
			printf("Position Joint 6 = %f\n", PositionCurrent[5]);

			printf("Current Joint 1 = %f\n", PositionCurrent[6]);
			printf("Current Joint 2 = %f\n", PositionCurrent[7]);
			printf("Current Joint 3 = %f\n", PositionCurrent[8]);
			printf("Current Joint 4 = %f\n", PositionCurrent[9]);
			printf("Current Joint 5 = %f\n", PositionCurrent[10]);
			printf("Current Joint 6 = %f\n", PositionCurrent[11]);
		}
		else
		{
			printf("Error during InitAPI.");
		}


		//We free the memory of the library.
		FreeLibrary(handleAPI);
	}
	
	getch();

	return 0;
}

