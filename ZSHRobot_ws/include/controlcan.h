#ifndef CONTROLCAN_H
#define CONTROLCAN_H
#pragma pack(1)
////文件版本：v2.02 20190609
// 接口卡类型定义

#define VCI_USBCAN1 3
#define VCI_USBCAN2 4
#define VCI_USBCAN2A 4

#define VCI_USBCAN_E_U 20
#define VCI_USBCAN_2E_U 21

// 函数调用返回状态值
#define STATUS_OK 1
#define STATUS_ERR 0

#define USHORT unsigned short int
// #define BYTE unsigned char
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void *
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void *
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

// 1.ZLGCAN系列接口卡信息的数据类型。
typedef struct _VCI_BOARD_INFO
{
	USHORT hw_Version;				// 硬件版本号
	USHORT fw_Version;				// 固件版本号
	USHORT dr_Version;				// 驱动程序版本号
	USHORT in_Version;				// 接口版本号
	USHORT irq_Num;					// 保留参数
	BYTE can_Num;					// 表示有记录CAN通道
	CHAR str_Serial_Num[20];		// 此版卡的序列号
	CHAR str_hw_Type[40];			// 硬件类型（USBCAN V1.00）
	USHORT Reserved[4];				// 系统保留
} VCI_BOARD_INFO, *PVCI_BOARD_INFO; // 接口卡信息数据类型

// 2.定义CAN信息帧的数据类型。
typedef struct _VCI_CAN_OBJ
{
	UINT ID;		 // 帧ID
	UINT TimeStamp;	 // 设备收集到某一帧的时间标识。时间标示从CAN卡上电开始计时，计时单位为0.1ms
	BYTE TimeFlag;	 // 是否时间使用时间标识，为1时TimeStamp有效，TimeFlag和TimeStamp只在此帧为接收帧时有效
	BYTE SendType;	 // 发送帧类型，=0 为正常发送(发送失败会自动重发，重发时间为4秒，4秒内没有发出则取消)， =1为单次发送(只发送一次，发送失败不会自动重发，总线只产生一帧数据)
	BYTE RemoteFlag; // 是否是远程帧(=0为数据帧，=1为远程帧(数据段空))
	BYTE ExternFlag; // 是否是扩展帧(=0 为标准帧(11位ID)，=1为扩展帧(29位ID))
	BYTE DataLen;	 // 数据长度DLC(<=8),即CAN帧Data有几个字节。约束后面Data[8]中的有效字节
	BYTE Data[8];	 // CAN帧的数据。由于CAN规定了最大是8个字节，所以这里预留了8字节的空间，受DataLen约束。如果是3，则Data[0]，Data[1]，Data[2]是有效的。
	BYTE Reserved[3];
} VCI_CAN_OBJ, *PVCI_CAN_OBJ; // 信息帧的数据类型

// 3.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG
{
	DWORD AccCode;
	DWORD AccMask;
	DWORD Reserved;
	UCHAR Filter;
	UCHAR Timing0;
	UCHAR Timing1;
	UCHAR Mode;
} VCI_INIT_CONFIG, *PVCI_INIT_CONFIG; // 定义初始化CAN的数据类型

//----------------------------------------//
typedef struct
{
	float vehicleLineVelocity;
	float vehicleAngularVelocity; // 前轮转角
	float Accerlerator;			  // 节气门 开的角度
	int BCARArmAngle;
} PatrolControllerMsg;

typedef struct
{
	uint16_t crc16;
	PatrolControllerMsg PatrolControllerMsg_;
} MsgHuman2RobPatrolFrame;
//----------------------------------------//

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD
{
	DWORD ExtFrame; // 是否为扩展帧
	DWORD Start;
	DWORD End;
} VCI_FILTER_RECORD, *PVCI_FILTER_RECORD;

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved);
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd);
EXTERN_C DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType, DWORD DeviceInd, PVCI_BOARD_INFO pInfo);

EXTERN_C DWORD VCI_SetReference(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);

EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

EXTERN_C ULONG VCI_Transmit(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, UINT Len);
EXTERN_C ULONG VCI_Receive(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, UINT Len, INT WaitTime);

EXTERN_C DWORD VCI_UsbDeviceReset(DWORD DevType, DWORD DevIndex, DWORD Reserved);
EXTERN_C DWORD VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

void *receive_func(void *param); // 接收线程。
void CAN_bus(void);
void UDP_busvel(void);

#endif
