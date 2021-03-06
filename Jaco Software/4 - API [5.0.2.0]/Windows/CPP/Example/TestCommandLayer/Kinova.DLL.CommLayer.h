
#ifdef KINOVADLLCOMMLAYER_EXPORTS
#define KINOVADLLCOMMLAYER_API __declspec(dllexport)
#else
#define KINOVADLLCOMMLAYER_API __declspec(dllimport)
#endif

//ERROR CODE
#define NO_ERROR 1
#define ERROR_LOAD_USB_LIBRARY 1001
#define ERROR_LOAD_USB_METHOD  1002
#define ERROR_GET_DEVICE_COUNT_METHOD  1003
#define ERROR_INIT_USB_METHOD  1004
#define ERROR_FREE_USB_METHOD  1005
#define ERROR_SEND_PACKET_METHOD  1006
#define ERROR_GET_DLL_VERSION_METHOD  1007
#define ERROR_OPEN_METHOD  1008
#define ERROR_WRITE_METHOD  1009
#define ERROR_READ_METHOD  1010
#define ERROR_READ_INT_METHOD  1011
#define ERROR_CLOSE_METHOD  1012
#define ERROR_FREE_LIBRARY  1013

#define PACKET_SIZE 64
#define PACKET_DATA_SIZE 56
#define PACKET_HEADER_SIZE 8

#define COMM_LAYER_VERSION 10000

struct Packet
{
	short IdPacket;
	short TotalPacketCount;
	short IdCommand;
	short TotalDataSize;
	unsigned char Data[PACKET_DATA_SIZE];
};

extern "C"
KINOVADLLCOMMLAYER_API int LoadUsbConnector(void);

extern "C"
KINOVADLLCOMMLAYER_API int CloseUsbConnector(void);

extern "C"
KINOVADLLCOMMLAYER_API DWORD GetDeviceCount(DWORD &result);

extern "C"
KINOVADLLCOMMLAYER_API DWORD InitUSB(DWORD ID);

extern "C"
KINOVADLLCOMMLAYER_API DWORD FreeUSB();

extern "C"
KINOVADLLCOMMLAYER_API Packet SendPacket(Packet &dataOut, Packet &dataIn, DWORD &result);
