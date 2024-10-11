/************************************************************
Module name: upc2_pci.h
************************************************************/
#ifdef __cplusplus
extern "C"   {
#endif

#if !defined(DllExport)
#define DllExport __declspec(dllexport)
#endif

// Prototypes 

// Test code

DllExport long __stdcall UPC2_PCI_UploadArray(long size, void * pFile);
DllExport long __stdcall UPC2_PCI_DownloadArray(long * pSize, void * pFile);

DllExport long __stdcall UPC2_PCI_Test(void);

// Internal support

void CheckFrameNo(void * pframe, long nFrames, U32 fsize);

void  BuildCRCTable(void);
U32   Calculate32BitCRC( long count, void * buffer );

long  GetStatus(long card_ndx);
long  GetMemoryMapPlus(long card_ndx);
long  GetConvertedDataFramePoolHdr(long card_ndx);

long  IsConnected(long card_ndx);
long  IsAwaitingCommand(long card_ndx);
void  SetCommandBuffer(U32 command, UPC2_CommandBuffer_t * pCommandBuffer);
long get_DSP_code_version(long card_ndx);

long  SelectPCI(DEVICE_LOCATION * pDevice, long n);
long  OpenPCI(DEVICE_LOCATION * pDevice, HANDLE *pDrvHandle);
long  Map_BAR(long card_ndx);
void  UnMap_BAR(long card_ndx);
long  ClosePCI(HANDLE DrvHandle);

DllExport long __stdcall WriteToLocalAddressSpace(long card_ndx, void * src, U32 local_addr, U32 size);
DllExport long __stdcall ReadFromLocalAddressSpace(long card_ndx, U32 local_addr, void * dest, U32 size);
DllExport long __stdcall ReadWithCheckFromLocalAddressSpace(long card_ndx, U32 local_addr, void * dest, U32 size);

// Vestigial
DllExport long __stdcall UPC2_PCI_WriteToLocalBus(long card_ndx, void * src, U32 dest, U32 size);
DllExport long __stdcall UPC2_PCI_ReadFromLocalBus(long card_ndx, U32 src, void * dest, U32 size);

// Command support
long SendCommandEx(long card_ndx, U32 command, long retry_mult);
long SendCommand(long card_ndx, U32 command, long retry_mult);

// Connect/Disconnect/Reset/SetTimestamp
DllExport long __stdcall UPC2_PCI_GetInventory(long * psn);
DllExport long __stdcall UPC2_PCI_Connect(long card_ndx);
DllExport long __stdcall UPC2_PCI_Disconnect(long card_ndx);
DllExport long __stdcall UPC2_PCI_Reset(long card_ndx);
DllExport long __stdcall UPC2_PCI_SetTimestamp(long timestamp);

// Data collection
DllExport long __stdcall UPC2_PCI_UploadConfig(long card_ndx, UPC2_Config_t * pUPC2_Config);
DllExport long __stdcall UPC2_PCI_UploadConfigFromPath(long card_ndx, char * pFilePath);
DllExport long __stdcall UPC2_PCI_StartDataCollection(long card_ndx);
DllExport long __stdcall UPC2_PCI_SetStartFrame(long card_ndx);
DllExport long __stdcall UPC2_PCI_GetUnreadFrameCount(long card_ndx);
DllExport long __stdcall UPC2_PCI_GetData(long card_ndx, long access_type, long nFrames, void * pFrame);
DllExport long __stdcall UPC2_PCI_StopDataCollection(long card_ndx);
DllExport long __stdcall UPC2_PCI_SaveConfigToFlash(long card_ndx);
DllExport long __stdcall UPC2_PCI_LoadConfigFromFlash(long card_ndx);
DllExport long __stdcall UPC2_PCI_GetNumberOfItems(long card_ndx);
DllExport long __stdcall UPC2_PCI_DownloadConfig(long card_ndx, UPC2_Config_t * addr);

// In-circuit programming
DllExport long __stdcall UPC2_PCI_InCircuitProgram(long card_ndx, long dest, char * pFilepath);
DllExport long __stdcall UPC2_PCI_DownloadProgram(long card_ndx, long src, void * pBuf);

// Production
DllExport long __stdcall UPC2_PCI_UploadCalibrationData(long card_ndx, UPC2_Calib_data_t * pUPC2_Calib_data);
DllExport long __stdcall UPC2_PCI_DownloadCalibrationData(long card_ndx, UPC2_Calib_data_t * pUPC2_Calib_data);
DllExport long __stdcall UPC2_PCI_SaveCalibrationDataToFlash(long card_ndx);
DllExport long __stdcall UPC2_PCI_LoadCalibrationDataFromFlash(long card_ndx);
DllExport long __stdcall UPC2_PCI_WriteSystemSerialNumber(long card_ndx, U32 sn);
DllExport long __stdcall UPC2_PCI_WriteSystemSerialNumberToEEPROM(long card_ndx, U32 sn);
DllExport long __stdcall UPC2_PCI_ReadSystemSerialNumber(long card_ndx, U32 *psn);
DllExport long __stdcall UPC2_PCI_ReadSystemSerialNumberFromEEPROM(long card_ndx, U32 *psn);

// Diagnostics
DllExport long __stdcall UPC2_PCI_GetSysInfo(long card_ndx, sw_info_t * pSwinfo, sw_info_t * pDLLinfo);
DllExport long __stdcall UPC2_PCI_GetSysOpInfo(long card_ndx, op_info_t * pOPinfo);
DllExport long __stdcall UPC2_PCI_RunDiagnostics(long card_ndx, long command, void * addr , long size, long * status);


#ifdef __cplusplus
}
#endif
//////////////////////// End Of File ////////////////////////
