
//
//  Name:
//
//    upc2_pci.c -- Interface DLL to UPC - 2 PCI version
//
//
// Description:
//
//    This nodule contains all the source code to interface with the 
//    PCI version of the UPC - 2
//
// Revisions:
//
// Contents:
//
///////////////////////////////////////////////////////////////////////////
//                          System functions
///////////////////////////////////////////////////////////////////////////
//
// hex2bin								- converts ASCII hex to binary
// parseHexLine     					- parses one line of Intel Hex
// DownloadHexFile 						- reads Intel hex file into pgm_buf
//
// UPC2_PCI_UploadArray	 				- tests writing a large array
// UPC2_PCI_DownloadArray				- tests reading a large array
// UPC2_PCI_Test						- performs tests during development
//
// BuildCRCTable						- builds table for CRC calculation
// Calculate32BitCRC					- calculates 32-bit CRC
//
// GetStatus 							- gets command status
//
// GetMemoryMapPlus						- gets a copy of SDRAM segment
// GetConvertedDataFramePoolHdr			- gets a copy of SDRAM segment
// IsConnected							- checks for card connected
// IsAwaitingCommand					- checks DSP operational state
// SetCommandBuffer						- sets up the command buffer
//
// SelectPCI 							- selects the n-th PLX device
// OpenPCI 								- opens a specific PCI device
// Map_BAR								- maps the Bas Address Register
// UnMap_BAR							- unmaps the Bas Address Register
// ClosePCI  							- closes a PCI device
//
// WriteToLocalAddressSpace 			- performs virtual write of SDRAM
// ReadFromLocalAddressSpace 			- performs virtual read	of SDRAM
//
// UPC2_PCI_WriteToLocalBus 			- (vestigial)
// UPC2_PCI_ReadFromLocalBus 			- (vestigial)
//
///////////////////////////////////////////////////////////////////////////
//                          Application functions
////////////////////////////////////////////////////////////////////////////
// Connect/Disconnect
// UPC2_PCI_GetInventory				- gets the number of PCI cards and their serial numbers
// UPC2_PCI_Connect						- connects to n-th PCI card
// UPC2_PCI_Disconnect              	- disconnects from n-th PCI card
//
// UPC2_PCI_Reset						- performs a reset of the PLX chip
// UPC2_PCI_SetTimestamp 				- sets timestamp
//
// Data Collection:					
//
// UPC2_PCI_UploadConfig 				- writes the UPC2_Config structure to SDRAM
// UPC2_PCI_UploadConfigFromPath 		- writes the UPC2_Config structure (from .cfg file) to SDRAM
// UPC2_PCI_StartDataCollection 		- sends a command to initiate data collection
// UPC2_PCI_SetStartFrame 				- sets the starting frame for GetData (UPC2_FROM_START_FRAME)
// UPC2_PCI_GetData 					- reads converted data from the ring buffer

// UPC2_PCI_GetCountUnreadFrames		- reads count of unread frames in the buffer

// UPC2_PCI_StopDataCollection 			- sends a command to terminate data collection
// UPC2_PCI_SaveConfigToFlash 			- saves the current configuration to Flash memory
// UPC2_PCI_LoadConfigFromFlash			- sends a command to load the configuration from Flash memory
// UPC2_PCI_GetNumberOfItems 			- gets the number of items from the current configuration
// UPC2_PCI_DownloadConfig 				- reads the current configuration

// In-circuit programming:				   
//
// UPC2_PCI_InCircuitProgram 			- programs the selected device using the binary file
// UPC2_PCI_DownloadProgram 			- reads the binary image of the program of the selected device

// Production support:						 
//
// UPC2_PCI_UploadCalibrationData		- writes the UPC2_Calibration_Data structure into the Channel
//                                      		  Adjustment Table in SDRAM
// UPC2_PCI_DownloadCalibrationData		- reads the UPC2_Calibration_Data structure from the Channel 
//                                        		  Adjustment Table in SDRAM
// UPC2_PCI_SaveCalibrationDataToFlash 	- sends a command to write the Channel Adjustment Table 
//                                             from SDRAM into flash memory
// UPC2_PCI_LoadCalibrationDataFromFlash - sends a command to load the UPC2_Calibration_Data structure
//                                             from flash memory into the Channel Adjustment Table in SDRAM
// UPC2_PCI_WriteSystemSerialNumber		- writes the system serial number to EEPROM  
// UPC2_PCI_ReadSystemSerialNumber 		- read the system serial number from EEPROM  
//
// Diagnostics:
//
// UPC2_PCI_GetSysInfo 	 		- gets system information specifically version, creation date and code size 
//                        			(in bytes) for the DSP and this DLL
// UPC2_PCI_GetSysOpInfo 		- gets system information specifically the local address of
// UPC2_PCI_RunDiagnostics 		- sends a command to run the commanded diagnostic.
//
//
///////////////////////////////////////////////////////////////////////////
//                          Include Files								  //
////////////////////////////////////////////////////////////////////////////
// For detecting memory leaks
//#define CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>

// For detecting memory leaks (insert this call in the code)
//_CrtDumpMemoryLeaks();

#include "PlxApi.h"
#include "PlxError.h"

#include "upc2_def.h"
#include "upc2_pci.h"
#include "parseHex.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

//#define LOG_ERROR
#define SIZE_BUFFER         0x100           // Number of bytes to transfer
#define SOFTWARE_HRDY	    0
#define USE_SEND_COMMAND
//////////////////////////////////////////////////////////////////////////////
//                            Global Variables                              //
//////////////////////////////////////////////////////////////////////////////
//U32 ddata[1024];		// for DEBUG
//U32 wdata[1024];		// for DEBUG

U32     newFrameAddr;

// for DEBUG
long    frame_no;
long    retry_count;
long    frame_count;

LARGE_INTEGER frq;		// for DEBUG

U8  pgm_buf[200000];
H_DATA hData;
UWB hexCodeChecksum;

long    pSize;

typedef struct 
{
	UPC2_SDRAM_MemoryMap_t   MemoryMap;
	UPC2_CommandBuffer_t     CommandBuffer;
} SDRAM_Image_t;

long cmd_status;

SDRAM_Image_t     SDRAM_Image;
UPC2_Calib_data_t Calib_Image;

UPC2_ConvertedDataFramePoolHdr_t FrameHdrImage;

// for DEBUG
U32 dframe[100];

UPC2_Config_t           UPC2_Config;

DEVICE_LOCATION         UPC2_Device[MAX_PCI_CARDS];
HANDLE                  UPC2_hDevice[MAX_PCI_CARDS];
U32                     UPC2_Va[MAX_PCI_CARDS];			  // virtual address
U32                     UPC2_Fac[MAX_PCI_CARDS];		  // offset factor

long            nPCI_cards;
long            UPC2_SysSerNum[MAX_PCI_CARDS];
long            UPC2_PCI_State[MAX_PCI_CARDS];
long            UPC2_DSP_State[MAX_PCI_CARDS];

// For Demo mode (i.e.not connected)

typedef struct
{
	Int32    nItems;
	Int32    nSbits;
	Int32    frame_no;
	Uint32   time_in_ms;
	Int32    scan_interval;
	float    scale[MAX_ITEMS];
	float    offset[MAX_ITEMS];
} demo_config_t;

demo_config_t demo_config[MAX_PCI_CARDS];


RETURN_CODE             UPC2_Orig_rc;				// return code from PLX API

U32 CRCTable[ 256 ];

// CRC test data
U32 CRCTest1[] = 
{
	0x12345678,
	0x23456789,
	0x34567890,
	0x45678901,
	0x56789012
};
U32 CRCTest2[] = 
{
	0x12345678,
	0x23456789,
	0x34567890,
	0x55678901,
	0x56789012
};

//////////////////////////////////////////////////////////////////////////////
//                            Code                                          //
//////////////////////////////////////////////////////////////////////////////
//
// DLLMain -- main sentry point
//
BOOL APIENTRY DllMain( HANDLE hModule, 
							  DWORD  ul_reason_for_call, 
							  LPVOID lpReserved
							)
{
	int i, j;
	BOOL retval = 0;

	if (ul_reason_for_call == DLL_PROCESS_ATTACH)
	{
#if 0


		// for DEBUG
		for (i = 0; i < 1024; i++)
			ddata[i] = i +0x12340000;
#endif


		// Initialize specific globals 
		newFrameAddr = 0;
		frame_no = 1;
        retry_count = 0;
		frame_count  = 0;


		UPC2_Orig_rc = 0;
		nPCI_cards = 0;

		for (i = 0; i < MAX_PCI_CARDS;i++)
		{
			UPC2_SysSerNum[i] = 0;
			UPC2_PCI_State[i] = 0;
			UPC2_DSP_State[i] = 0;
			demo_config[i].nItems = 0;
			demo_config[i].nSbits = 0;
			demo_config[i].scan_interval = 0;
			demo_config[i].frame_no = 0;
			demo_config[i].time_in_ms = 0;
			for (j = 0; j < MAX_ITEMS; j++)
			{
				demo_config[i].scale[j] = 1.0;
				demo_config[i].offset[j] = 0;
			}
		}

		// Seed the random-number generator with current time so that
		// the numbers will be different every time we run.

		// srand( (unsigned)time( NULL ) );

		BuildCRCTable();
	}
	else if (ul_reason_for_call == DLL_PROCESS_DETACH)
	{
		// Disconnect all connected devices
		for (i = 0; i < MAX_PCI_CARDS; i++)
		{
			if (IsConnected(i) > 0)
			{
				UPC2_PCI_Disconnect(i);
			}
		}

	}
	retval = QueryPerformanceFrequency(&frq);
	return TRUE;	  // OK
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//   Name:         hex2bin
//
//   Description:  Converts 2 ascii hex characters to binary
//
//   Parameters:   msb, lsb  The data to convert
//        
//   Returns:      0 -> 0xff or 0xffff if invalid hex
//
/////////////////////////////////////////////////////////////////////////////////////////////////
INT hex2bin(UCHAR msb, UCHAR lsb)
{
	if ((msb >= '0') && (msb <= '9'))
	{
		msb -= '0';
	}
	else
	{
		if ((msb >= 'A') && (msb <= 'F'))
		{
			msb -= ('A' - 10);
		}
		else
		{
			return(0xffff);
		}
	}

	if ((lsb >= '0') && (lsb <= '9'))
	{
		lsb -= '0';
	}
	else
	{
		if ((lsb >= 'A') && (lsb <= 'F'))
		{
			lsb -= ('A' - 10);
		}
		else
		{
			return(0xffff);
		}
	}
	return((msb * 16) + lsb);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
//   Name:         parseHexLine
//
//   Description:  Parses one line of Intel Hex Code and places information in the H_DATA record. 
//                 
//                 'addr' is the address of the first byte in 'data' and is is always correct based
//                 on the Intel protocol for extended linear addressing.  
//
//                 Currently only the 00 data record, 04 extended linear addressing, and 
//                 01 end of file records are supported.
//
//   Parameters:   pStr  Pointer to ASCII string containing Intel Hex Code.
//        
//   Returns:      H_Status record
//
/////////////////////////////////////////////////////////////////////////////////////////////////
H_STATUS parseHexLine(CHAR *pStr)
{
	INT       len = strlen(pStr);
	UINT      i,checksum;
	INT       idx,idxEnd,byteNo;
	H_STATUS  s;

	/* check that we have enough bytes in the string to get us started */
	if (len < 12)
	{
		return(H_NO_RECORD);
	}
	/* first byte must be ':' return if not */
	if (pStr[0] != ':')
	{
		return(H_START);
	}
	/* next byte is the number of data bytes characters */
	if ((i = hex2bin(pStr[1],pStr[2])) > 255)
	{
		return(H_HEX_ERROR);
	}
	checksum = i;
	hData.number_bytes = (UCHAR)i;
	idxEnd = i * 2 + 11;
	/* next get the load offset */
	if ((i = hex2bin(pStr[3],pStr[4])) > 255)		/* high byte */
	{
		return(H_HEX_ERROR);
	}
	checksum += i;
	hData.addr.b.lm = (UCHAR)i;
	if ((i = hex2bin(pStr[5],pStr[6])) > 255)		/* low byte */
	{
		return(H_HEX_ERROR);
	}
	checksum += i;
	hData.addr.b.ll = (UCHAR)i;
	/* now get the record type and determine what to do based on it */
	if ((i = hex2bin(pStr[7],pStr[8])) > 255)		/* low byte */
	{
		return(H_HEX_ERROR);
	}
	checksum += i;
	idx = 9;
	switch (i)
	{
		case 0:		/* data record */
			for (byteNo=0;byteNo < hData.number_bytes;byteNo++)
			{
				if (idx < idxEnd)
				{
					if ((i = hex2bin(pStr[idx],pStr[idx+1])) > 255)
					{
						return(H_HEX_ERROR);
					}
					checksum += i;
					hexCodeChecksum.u += i;
					hData.data[byteNo] = (UCHAR)i;
					idx += 2;          
				}
				else
				{
					return(H_NO_RECORD);
				}
			}
			s = H_00;
			break;
		case 1:		/* end record */
			s = H_01;
			break;
		case 4:		/* extended linear address record */
			if ((idx + 1) < idxEnd)
			{
				/* get extended linear address */
				if ((i = hex2bin(pStr[idx],pStr[idx+1])) > 255)		/* high byte */
				{
					return(H_HEX_ERROR);
				}
				checksum += i;
				hData.addr.b.mm = (UCHAR)i;
				if ((i = hex2bin(pStr[idx+2],pStr[idx+3])) > 255)	  /* low byte */
				{
					return(H_HEX_ERROR);
				}
				checksum += i;
				hData.addr.b.ml = (UCHAR)i;
				idx += 4;
				s = H_04;
			}
			else
			{
				return(H_NO_RECORD);
			}
			break;
	}
	/* if there is still data left, then test checksum */
	if ((idx + 1) < idxEnd)
	{
		/* test checksum */
		if ((i = hex2bin(pStr[idx],pStr[idx+1])) > 255)		/* low byte */
		{
			return(H_HEX_ERROR);
		}
		if (((i + checksum) & 0xff) != 0)
		{
			return(H_INVALID_CHECKSUM);
		}
	}
	else
	{
		return(H_NO_RECORD);
	}
	return(s);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// DownloadHexFile -- Reads Intel hex file into pgm_buf
//
// parameters:
//
// pFilePath -- pointer to a file path string
//
// Returns -- size in bytes of downloaded image or
//            
//            negative if an error occurs:
//			  		    UPC2_FILE_OPEN_ERR
//			  		    UPC2_CKSUM_ERR
//			  		    UPC2_HEX_FILE_ERR
//
long DownloadHexFile(char * pFilePath)
{
	FILE *handle;
	long ret_val = 0;
	long pgm_size = 0;
	INT line = 0;
	INT idx;
	H_STATUS s;
	char line_buf[256];
	char str[80];

	// Open file
	if ((handle = fopen(pFilePath, "r")) == NULL)
	{
		OutputDebugString("Unable to open file");
		ret_val = UPC2_FILE_OPEN_ERR;
		return ret_val;
	}

	// Init buffer
	memset(pgm_buf, -1, sizeof(pgm_buf));

	// MaxG 9-08-05 Init hData to clear bit 16
	hData.addr.u = 0;

	// Read file into buffer (calculating code size)
	while (fgets(line_buf, sizeof(line_buf), handle) != NULL)
	{
		line++;
		switch (s = parseHexLine(line_buf))
		{
			case H_00:							  /* Valid 00 data record */
				// Transfer data to pgm_buf
				if ((hData.addr.u + (unsigned long)hData.number_bytes) < sizeof(pgm_buf))
				{
					/* its OK so copy it */
					for (idx=0;idx < (long)hData.number_bytes;idx++)
					{
						pgm_buf[hData.addr.u + idx] = hData.data[idx];
					}
					pgm_size += hData.number_bytes; 
				}
				break;
			case H_01:							  /* Valid 01 end record */
				break;
			case H_04:							  /* Valid 04 extended linear record */
				break;
			case H_UNKNOWN:				 /* Unkown record type */
				sprintf(str,"Unknown record type at line %d",line);
				OutputDebugString(str);
				break;
			case H_START:						  /* no ':' found */
				sprintf(str,": not starting character at line %d",line);
				OutputDebugString(str);
				break;
			case H_NO_RECORD:					  /* not enough bytes for a record */
				sprintf(str,"Not enough bytes in record at line %d",line);
				OutputDebugString(str);
				break;
			case H_HEX_ERROR:					  /* hex error */
				sprintf(str,"Hex data not found at line %d",line);
				OutputDebugString(str);
				break;
			case H_INVALID_CHECKSUM:	 /* Invalid checksum */
				sprintf(str,"Invalid checksum at line %d",line);
				OutputDebugString(str);
				ret_val = UPC2_CKSUM_ERR;
				break;
			default:
				sprintf(str,"Unknown error at line %d",line);
				OutputDebugString(str);
				s = H_UNKNOWN_ERROR;
		}
		if ((s > H_04) && (ret_val == 0))
			ret_val = UPC2_HEX_FILE_ERR;
	}
	fclose(handle);

	if ((s == H_01) && (ret_val == 0))
		ret_val = pgm_size;

	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_UploadArray 
//
// parameters:
//
// size -- size of download (in bytes)
//
// pFile -- pointer to a buffer to upload
//
//
DllExport long __stdcall UPC2_PCI_UploadArray(long size, void * pFile)
{
	if (size <= sizeof(pgm_buf))
	{
		memcpy(pgm_buf, pFile, size);
		return UPC2_NORMAL_RETURN;
	}
	else
		return UPC2_COMM_ERR;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_DownloadArray 
//
// parameters:
//
// pSize -- pointer to a long that contains size (in bytes) of download
//
// pFile -- pointer to a buffer for download
//
// Returns -- negative if an error occurs. 
//
//
DllExport long __stdcall UPC2_PCI_DownloadArray(long * pSize, void * pFile)
{
	U32  size = 50000;
	U32  i;
	U32  * px;

	for (i = 0; i < 50000; i+=4)
	{
		px = (U32 *)&pgm_buf[i];
		*px = i;
	}
	*pSize = size;
	memcpy(pFile, pgm_buf, size);
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_Test reads a cfg file
//
// parameters:
//
// none
//
// Returns -- negative if an error occurs. 
//
//
DllExport long __stdcall UPC2_PCI_Test(void)
{
	FILE     *handle;
	long     ret_val = 1;
	size_t   numread;
	char     flpath[] = "c:\\test.cfg";

	// Open file

	if ((handle = fopen(flpath, "rb")) == NULL)
	{
		OutputDebugString("Unable to open file");
		ret_val = UPC2_FILE_OPEN_ERR;
		return ret_val;
	}
	numread = fread((void *)&UPC2_Config, sizeof(UPC2_Config_t), 1, handle);
	fclose(handle);
	return ret_val;
}
#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_Test writes and reads PLX RDK 9030 Lite board
//
// parameters:
//
// none
//
// Returns -- negative if an error occurs. 
//
//
DllExport long __stdcall UPC2_PCI_Test(void)
{
	//DEVICE_LOCATION 	Device;
	//HANDLE          	hDevice;
	long             n;
	long             ret_val;
	long             i;
	long             LocalAddress = SDRAM_BASE;
	long             *pBufferDest;
	long             *pBufferSrc;

	U32              crc1 = 0;
	U32              crc2 = 0;
	U32              crc3, crc4;

	ret_val = UPC2_BAD_CRC;
	UPC2_Orig_rc +=1;

	BuildCRCTable();

	crc1 = Calculate32BitCRC( sizeof(CRCTest1), CRCTest1);
	crc2 = Calculate32BitCRC( sizeof(CRCTest2), CRCTest2);

	crc3 = Calculate32BitCRC( sizeof(CRCTest1), CRCTest1);
	crc4 = Calculate32BitCRC( sizeof(CRCTest2), CRCTest2);

	if (crc1 != crc3 || crc2 != crc4)
		return ret_val;

	for (;;)
	{
		// Connect to n-th PLX PCI device
		n = 0;
		if ((ret_val = UPC2_PCI_Connect(n)) < 0)
			break;

		// Allocate buffers
		pBufferDest = (long*)malloc(SIZE_BUFFER);
		if (pBufferDest == NULL)
		{
			ret_val = UPC2_COMM_ERR-20;
			break;
		}

		pBufferSrc  = (long*)malloc(SIZE_BUFFER);
		if (pBufferSrc == NULL)
		{
			ret_val = UPC2_COMM_ERR-21;
			break;
		}

		// Prepare buffer data
		for (i=0; i < (SIZE_BUFFER >> 2); i++)
			pBufferSrc[i] = i;

		// Clear destination buffer
		memset(
				pBufferDest,
				0,
				SIZE_BUFFER
				);

		// Write to SDRAM
		WriteToLocalAddressSpace(n, pBufferSrc, LocalAddress, SIZE_BUFFER);

		// Read from SDRAM
		ReadFromLocalAddressSpace(n, LocalAddress, pBufferDest, SIZE_BUFFER);
		break;

		// Verify read
		if (memcmp(
					 pBufferSrc,
					 pBufferDest,
					 SIZE_BUFFER
					 ) != 0)
		{
			ret_val = UPC2_COMM_ERR-22;
			break;
		}
		ret_val = UPC2_NORMAL_RETURN;
		break;
	}

	// Free Buffers
	if (pBufferDest != NULL)
		free(pBufferDest);

	if (pBufferSrc != NULL)
		free(pBufferSrc);

	// Disconnect PCI device
	ret_val = UPC2_PCI_Disconnect(n);

	return ret_val;
}
#endif
//////////////////////////////////////////////////////////////////////////////
//                   Functions for internal use								//
//////////////////////////////////////////////////////////////////////////////
//  DDJ 1992 Mark Nelson
// 
//  BuildCRCTable -- builds table for use by Calculate32BitCRC
//
void BuildCRCTable(void)
{
	long i;
	long j;
	U32 crc;

	for ( i = 0; i <= 255 ; i++ )
	{
		crc = i;
		for ( j = 8 ; j > 0; j-- )
		{
			if ( crc & 1 )
				crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
			else
				crc >>= 1;
		}
		CRCTable[ i ] = crc;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Calculate32BitCRC -- calculates the 32-bit CRC for a block of data
//
// parameters:
//
// count 	-- count (in bytes)
// buffer 	-- pointer to a block of data
//
// Returns  -- 32-bit CRC
//  
// The original version accepted an initial value for the crc and returned an updated value.
// U32 CalculateBufferCRC( long count, U32 crc, void * buffer )
//
U32 Calculate32BitCRC(long count, void * buffer )
{
	U8  *p;
	U32 crc = 0;
	U32 temp1;
	U32 temp2;

	p = (U8 *) buffer;
	while ( count-- != 0 )
	{
		temp1 = ( crc >> 8 ) & 0x00FFFFFFL;
		temp2 = CRCTable[ ( (U32) crc ^ *p++ ) & 0xff ];
		crc = temp1 ^ temp2;
	}
	return crc;
}
///////////////////////////////////////////////////////////////////////////
//
// Calculate32BitChecksum -- calculates the 32-bit checksum for a block of data
//
// parameters:
//
// count 	-- count (in 32-bit words)
// buffer 	-- pointer to a block of data
//
// Returns  -- 32-bit checksum
//  
//
U32 Calculate32BitChecksum(long count, U32 * buffer )
{
	Uint32  *p;
	Uint32  checksum = 0;

	p = buffer;
	while ( count-- != 0 ) {
		checksum += *p++;
	}
	return checksum;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  GetStatus -- Gets command status
//
//  Returns -- negative if an error occurs
//			-- positive if status
//
long GetStatus(long card_ndx)
{
	if (IsConnected(card_ndx) < 0)
		return UPC2_NO_CONNECTION;

	ReadFromLocalAddressSpace(card_ndx, (U32) COMMAND_BUFFER_ADDR + sizeof(UPC2_CommandBuffer_t) - 4,
									  &cmd_status, sizeof(cmd_status));
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  GetMemoryMapPlus -- Gets the SDRAM memory map and command buffer
//
//  Returns -- negative if an error occurs
//			-- positive if SDRAM_Image loaded
//
long GetMemoryMapPlus(long card_ndx)
{
	if (IsConnected(card_ndx) < 0)
		return UPC2_NO_CONNECTION;

	ReadFromLocalAddressSpace(card_ndx, (U32) SDRAM_MEMORY_MAP_ADDR,
									  &SDRAM_Image, sizeof(SDRAM_Image));
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  GetConvertedDataFramePoolHdr --
//
//  Returns -- negative if an error occurs
//			-- positive if FrameHdrImage loaded
//
long GetConvertedDataFramePoolHdr(long card_ndx)
{
	if (IsConnected(card_ndx) < 0)
		return UPC2_NO_CONNECTION;

	ReadFromLocalAddressSpace(card_ndx, CONVERTED_DATA_FRAMES_POOL_HDR_ADDR,
									  &FrameHdrImage, sizeof(FrameHdrImage));
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//  IsConnected --  tests for card connected
//
//  Returns -- negative if error
//					UPC2_INVALID_INDEX
//             		UPC2_NO_CONNECTION
//			   positive if connected
//  				UPC2_CONNECTED		
//
//
long IsConnected(long card_ndx)
{
	if (card_ndx > MAX_PCI_CARDS || card_ndx < 0)
		return UPC2_INVALID_INDEX;

	if (UPC2_PCI_State[card_ndx] & UPC2_CONNECTED)
		return UPC2_CONNECTED;
	else
		return UPC2_NO_CONNECTION;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//  IsAwaitingCommand -- tests DSP operational state 
//
// 	Returns -- negative if an error occurs
//            UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP is busy
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION 	if not connected
//
long IsAwaitingCommand(long card_ndx)
{
	long ret_val;

	if ((ret_val = GetMemoryMapPlus(card_ndx)) < 0)
		return ret_val;

	if (SDRAM_Image.CommandBuffer.control_status & UPC2_DSP_BUSY)
		return UPC2_BUSY;
	else
		return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//  SetCommandBuffer -- sets up command buffer with command and CRC
//
void SetCommandBuffer(U32 command, UPC2_CommandBuffer_t * pCommandBuffer)
{
	memset(pCommandBuffer, 0, sizeof(UPC2_CommandBuffer_t));
	pCommandBuffer->command = command;
	pCommandBuffer->control_status = UPC2_DSP_NEW_COMMAND;
	pCommandBuffer->CRC = Calculate32BitCRC(sizeof(UPC2_CommandBuffer_t) - 4, ((U8 *)pCommandBuffer) + 4);
	return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
// SelectPCI selects the n-th PLX device
//
// parameters:
//
// pDevice -- pointer to a DEVICE_LOCATION struct
// n       -- index of device to be selected, 0, 1, 2, ...
//
// Returns -- negative if an error occurs. 
//   				UPC2_NO_VALID_DEVICE		if unable to locate any valid devices
//					UPC_NO_PLX_IN_NTH_POSITION 	if PLX device in n-th position
//				or
//
//			  -- number of devices otherwise
//
long SelectPCI(DEVICE_LOCATION * pDevice, long n)
{
	S32         ReqLimit;
	long        i;
	RETURN_CODE rc;


	// Find the number of PLX devices
	ReqLimit = FIND_AMOUNT_MATCHED;

	// No search criteria, select all devices
	pDevice->BusNumber  = 0xff;
	pDevice->SlotNumber = 0xff;
	pDevice->VendorId   = 0xffff;
	pDevice->DeviceId   = 0xffff;
	strcpy(pDevice->SerialNumber, "");

	rc = PlxPciDeviceFind(pDevice, &ReqLimit);
	UPC2_Orig_rc = rc;		// for Debug

	if (rc == ApiNoActiveDriver)			// Error - no driver
		return UPC2_NO_ACTIVE_DRIVER;

	if ((rc != ApiSuccess || ReqLimit  == 0))
	{
		return UPC2_NO_VALID_DEVICE;	// Error - unable to locate any valid devices
	}

	// Search for the n-th PLX device
	for (i=0; i<ReqLimit; i++)
	{
		pDevice->BusNumber  = 0xff;   
		pDevice->SlotNumber = 0xff;   
		pDevice->VendorId   = 0x10b5;	  // PLX Vendor ID
		pDevice->DeviceId   = 0x9030; 
		strcpy(pDevice->SerialNumber, "");

		UPC2_Orig_rc = rc;		// for Debug
		rc = PlxPciDeviceFind(pDevice, &i);
		if (rc != ApiSuccess)
			return UPC2_NO_PLX_IN_NTH_POSITION;	 // Error - unable to device in n-th position
		if (i == n)
		{
			// Found device - return with device information filled in
			return ReqLimit;
		}
	}

	//return error;
	return UPC2_NO_PLX_IN_NTH_POSITION;	 // Error - unable to device in n-th position
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// OpenPCI -- opens a specific PCI device
//
// parameters:
//
// pDevice    -- pointer to a DEVICE_LOCATION struct
// pDrvHandle -- pointer to a device handle
//
// Returns -- negative if an error occurs. 
//
long OpenPCI(DEVICE_LOCATION * pDevice, HANDLE *pDrvHandle)
{
	RETURN_CODE     rc;

	rc = PlxPciDeviceOpen(pDevice, pDrvHandle);

	if (rc != ApiSuccess)
	{
		// Error - unable to open device
		return UPC2_NO_VALID_DEVICE;
	}

	//return TRUE;
	return UPC2_NORMAL_RETURN;        
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// Map_BAR -- maps the BAR
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs. 
//
#pragma optimize("",off)
long Map_BAR(long card_ndx)
{
	U32              Va;
    U32              i;
	    
    U32              j,k;
	U32              Fac; 

	long  			  ret_val = UPC2_NORMAL_RETURN;

	PlxPciBarMap(
					UPC2_hDevice[card_ndx],
					3,			 // BarIndex 3 => space 1
					&Va
					);
	

	// MaxG 3-8-10 Reinstated PCI Reset to get operational on Win2K box
	// Reset n-th PLX PCI device
	PlxPciBoardReset(UPC2_hDevice[card_ndx]);

	// Delay ~ one second
	Sleep(1000);

	// Verify virtual address
	if (Va == (U32)-1 || Va == (U32)NULL)
	{
		return UPC2_COMM_ERR;					
	}

	// Setup HPIC
	*(U32*)(Va) = 0x00010001;	  // = HWOB = 1 => first halfword is least significant

	UPC2_Va[card_ndx] = Va;
	//return TRUE;
    
    
    // MaxG 6-18-09 Determine if HPI mapped for 2282 DMA by reading HPIC
    i = *(U32*)(Va);
    //j = *(U32*)(Va + 0x100);
	 j = *(U32*)(Va + 0x10);

    Fac = 0x200;              // Assume modifed card
    if (i != j)
       Fac = 1;               // Card is unmodifed

    UPC2_Fac[card_ndx] = Fac;	 
    
    // MaxG 6-18-09 Extended test
    
    // Test 1: Write 512 bytes to SDRAM and read them back
     
	*(U32*)(Va + 4*Fac) = 0xa0000000;     
		 
	for (i = 0; i < 512; i+=4)
	{
	  *(U32*)(Va + 8*Fac) = i;  // Write pattern
	}
		 
	*(U32*)(Va + 4*Fac) = 0xa0000000;
	 
	for (i = 0; i < 512; i+=4)
	{
	  j = *(U32*)(Va + 8*Fac);           // Read it back
	  if (j != i)
	  {
	     ret_val = UPC2_UNABLE_TO_MAP_BAR;
	   	 break;
	  }
	}

    // Test 2: Write 512 bytes to Internal RAM and read them back
     
    if (ret_val == UPC2_NORMAL_RETURN)
    {
	  *(U32*)(Va + 4*Fac) = 0x40000 - 512;   // 0x40000 = End of Internal RAM 
		 
	  for (i = 0; i < 512; i+=4)
	  {
	     *(U32*)(Va + 8*Fac) = i;  // Write pattern
	  }
		 
	  *(U32*)(Va + 4*Fac) = 0x40000 - 512;   // setting HPIA flushes write buffer
	 
	  for (i = 0; i < 512; i+=4)
	  {
	     j = *(U32*)(Va + 8*Fac);           // Read it back
	     if (j != i)
	     {
	        ret_val = UPC2_UNABLE_TO_MAP_BAR;
	   	    break;
	     }
	  }
    }
    return ret_val;        
}
#pragma optimize("",on)
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UnMap_BAR -- unmaps the BAR
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
void UnMap_BAR(long card_ndx)
{
	PlxPciBarUnmap(
					  UPC2_hDevice[card_ndx],
					  &UPC2_Va[card_ndx]
					  );
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// ClosePCI -- closes a PCI device
//
// parameters:
//
// DrvHandle -- handle of an open device
//
// Returns -- negative if an error occurs. 
//
long ClosePCI(HANDLE DrvHandle)
{
	RETURN_CODE     rc;

	rc = PlxPciDeviceClose(DrvHandle);
	UPC2_Orig_rc = rc;		 // for Debug

	if (rc != ApiSuccess)
	{
		// Error - unable to release device
		return -1;
	}

	//return TRUE;
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_WriteToLocalBus -- (vestigial)
//
// parameters:
//
// DrvHandle  -- Device handle
// src        -- pointer to source buffer
// dest       -- local address (absolute) 
// size       -- number of bytes to write
//
// Returns -- negative if an error occurs. 
//
DllExport long __stdcall UPC2_PCI_WriteToLocalBus(long card_ndx, void * src, U32 dest, U32 size)
//DllExport long __stdcall UPC2_PCI_WriteToLocalBus(HANDLE DrvHandle, void * src, U32 dest, U32 size)
{
	RETURN_CODE     rc;

	rc = PlxBusIopWrite(
							 //DrvHandle,
							 UPC2_hDevice[card_ndx],
							 //IopSpace0,
							 //MsLcs1,
							 IopSpace1,
							 dest,					// relative local addressx
							 FALSE,					// Re-map local window
							 src,
							 size,
							 BitSize8
							 );

	if (rc != ApiSuccess)
	{
		// Error - unable to write data
		return -1;
	}
	//return TRUE;
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_ReadFromLocalBus -- (vestigial)
//
// parameters:
//
// DrvHandle  -- Device handle
// src        -- local address (absolute) 
// dest       -- pointer to destination buffer
// size       -- number of 32-bit words to write
//
// Returns -- negative if an error occurs. 
//                                                            
DllExport long __stdcall UPC2_PCI_ReadFromLocalBus(long card_ndx, U32 src, void * dest, U32 size)
//DllExport long __stdcall UPC2_PCI_ReadFromLocalBus(HANDLE DrvHandle, U32 src, void * dest, U32 size)
{
	RETURN_CODE     rc;

	rc = PlxBusIopRead(
							//DrvHandle,
							UPC2_hDevice[card_ndx],
							IopSpace1,
							//IopSpace0,
							src,					 // relative local address
							FALSE,				 // Re-map local window
							dest,
							size,
							BitSize32
							);

	UPC2_Orig_rc = rc;		// for Debug
	if (rc != ApiSuccess)
	{
		// Error;
		return -1;
	}
	//return TRUE;
	return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// WriteToLocalAddressSpace -- performs virtual write of SDRAM
//
// parameters:
//
// card_ndx   -- long 0, 1, 2, .. representing the card's index
// local_addr -- local address 
// src        -- pointer to source buffer
// size       -- number of bytes to write (must be a multiple of 4)
//
// Returns -- negative if an error occurs
//			  UPC2_COMM_ERR	if HRDY doesn't go high 
// 
DllExport long __stdcall WriteToLocalAddressSpace(long card_ndx, void * src, U32 local_addr, U32 size)
{
   U32          Va = UPC2_Va[card_ndx]; 
	U32          Fac = UPC2_Fac[card_ndx]; 
	U32          i,j;
	U32          wrt_size = 32;
	char         str[100];
   
	//LARGE_INTEGER t0, t1, frq, tm, tf, tg;
	//double		dtm, dtf, dtg;

	//QueryPerformanceFrequency(&frq);
	//QueryPerformanceCounter(&t0);

	//QueryPerformanceCounter(&t1);
	//tm.QuadPart = t1.QuadPart-t0.QuadPart;

	while (size)
	{
		// Setup HPIA
		for (i = 0; i < 10000; i++)
		{
			if (local_addr >= 0x60000000 && local_addr <= 0x7fffffff)
			{
				sprintf(str, "Bad address request %x",local_addr);
				OutputDebugString(str);
				return UPC2_COMM_ERR;
			}
#if SOFTWARE_HRDY != 0	
			// Wait for HRDY bit to be one
			for (j = 0; j < 100; j++)
			{
				if ((*(U32*)Va & 0x00000008) != 0)
					break;
			}
			if (j == 100)
				return UPC2_COMM_ERR;
#endif
			*(U32*)(Va + 4*Fac) = local_addr; 
			if (*(U32*)(Va + 4*Fac) == local_addr)
				break;
		}

		if (size < 40)
			wrt_size = size;

		// Write data to HPID
		//for (i = 0; i < wrt_size; i += 4)
		for (i = 0; i < size; i += 4)
		{
#if SOFTWARE_HRDY != 0

			// Wait for HRDY bit to be one
			for (j = 0; j < 100; j++)
			{
				if ((*(U32*)Va & 0x00000008) != 0)
					break;
			}
			if (j == 100)
				return UPC2_COMM_ERR;
#endif			


			// SPRZ173M workaround: do a fixed-mode access on the last access
#ifdef SPRZ173M
			if ( i < (size - 4))
				*(U32*)(Va + 8*Fac) = *(U32*)((U32)src + i);
			else
				*(U32*)(Va + 0xc*Fac)	= *(U32*)((U32)src + i);
#else
			*(U32*)(Va + 8*Fac) = *(U32*)((U32)src + i);
#endif
		}
		//(U32)src += wrt_size;
		//local_addr+= wrt_size;
		//size -= wrt_size;
		size = 0;
	}
	//QueryPerformanceCounter(&t0);
	//tf.QuadPart = t0.QuadPart-t1.QuadPart;

	//for (i=0; i<size; i += sizeof(U32))
	// {
	//     *(U32*)((U32)src + offset + i) = *(U32*)((U32)src + i);
	// }
	//QueryPerformanceCounter(&t1);
	//tg.QuadPart = t1.QuadPart-t0.QuadPart;

	//dtm= (double)tm.QuadPart/(double)frq.QuadPart;
	//dtf= (double)tf.QuadPart/(double)frq.QuadPart;
	//dtg= (double)tg.QuadPart/(double)frq.QuadPart;

	// MaxG 9-9-08 Added setting HPIA to flush write buffer
	//    See HPI Guide sec 4.2.3
	*(U32*)(Va + 4*Fac) = local_addr;

	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// ReadFromLocalAddressSpace -- performs virtual read of SDRAM
//
// parameters:
//
// card_ndx   -- long 0, 1, 2, .. representing the card's index
// local_addr -- local address 
// dest       -- pointer to destination buffer
// size       -- number of bytes to read (must be a multiple of 4)
//
// Returns -- negative if an error occurs
//			  UPC2_COMM_ERR	if HRDY doesn't go high
//
DllExport long __stdcall ReadFromLocalAddressSpace(long card_ndx, U32 local_addr, void * dest, U32 size)
{
	U32          Va = UPC2_Va[card_ndx];
	U32          Fac = UPC2_Fac[card_ndx]; 
	U32          i,j;
	U32          rd_size = 32;
	char         str[100];

	//LARGE_INTEGER t0, t1, frq, tm, tf, tg;
	//double		dtm, dtf, dtg;

	//QueryPerformanceFrequency(&frq);
	//QueryPerformanceCounter(&t0);

	//QueryPerformanceCounter(&t1);
	//tm.QuadPart = t1.QuadPart-t0.QuadPart;
	//QueryPerformanceCounter(&t0);

    Va = UPC2_Va[card_ndx];

	// Setup HPIA
	if (local_addr >= 0x60000000 && local_addr <= 0x7fffffff)
	{
		sprintf(str, "Bad address request %x",local_addr);
		OutputDebugString(str);
		return UPC2_COMM_ERR;
	}

	*(U32*)(Va + 4*Fac) = local_addr;
	j = *(U32*)(Va + 4*Fac);		// check
	// Read data from HPID (auto-increment)
	for (i = 0; i < size; i += 4)
	{
#if SOFTWARE_HRDY != 0
		// Wait for HRDY bit to be one
		for (j = 0; j < 100; j++)
		{
			if ((*(U32*)Va & 0x00000008) != 0)
				break;
		}
		if (j == 100)
			return UPC2_COMM_ERR;
#endif
		//*(U32*)((U32) dest + i) = *(U32*)(Va + 0xc);		// fixed-mode access
		*(U32*)((U32) dest + i) = *(U32*)(Va + 0x8*Fac);				  // auto-increment mode
	}
	//QueryPerformanceCounter(&t1);
    //tg.QuadPart = t1.QuadPart - t0.QuadPart - tm.QuadPart;
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// ReadWithCheckFromLocalAddressSpace -- performs virtual read of SDRAM and verifies check word
//
// parameters:
//
// card_ndx   -- long 0, 1, 2, .. representing the card's index
// local_addr -- local address 
// dest       -- pointer to destination buffer
// size       -- number of bytes to read (including checksum) (must be a multiple of 4)
//
// Returns -- negative if an error occurs
//			  UPC2_COMM_ERR	if HRDY doesn't go high
//			  UPC2_CKSUM_ERR if unable to verify check word after READ_DATA_MAX_TRIES
//
DllExport long __stdcall ReadWithCheckFromLocalAddressSpace(long card_ndx, U32 local_addr, void * dest, U32 size)
{
	U32          Va = UPC2_Va[card_ndx];
	U32          Fac = UPC2_Fac[card_ndx]; 
	U32          i,j;
	U32          ck_word, cw;
	char         str[100];

	//for (k = 0; k < READ_DATA_MAX_TRIES; k++)
	//{
		// Setup HPIA
		if (local_addr >= 0x60000000 && local_addr <= 0x7fffffff)
		{
			sprintf(str, "Bad address request %x",local_addr);
			OutputDebugString(str);
			return UPC2_COMM_ERR;
		}

		*(U32*)(Va + 4*Fac) = local_addr;

		// Read data from HPID (auto-increment)
		for (i = 0; i < size -  4; i += 4)
		{
#if SOFTWARE_HRDY != 0
			// Wait for HRDY bit to be one
			for (j = 0; j < 100; j++)
			{
				if ((*(U32*)Va & 0x00000008) != 0)
					break;
			}
			if (j == 100)
				return UPC2_COMM_ERR;
#endif
			//*(U32*)((U32) dest + i) = *(U32*)(Va + 0xc);		// fixed-mode access
			*(U32*)((U32) dest + i) = *(U32*)(Va + 0x8*Fac);				  // auto-increment mode
		}
		// Read check word and compare to calculated value
		ck_word = *(U32*)(Va + 0xc*Fac);		// fixed-mode access
		cw = Calculate32BitChecksum((size - 4)/4, (U32 *) dest);
        
        // For Teledyne
		if (cw != ck_word)
            return UPC2_CKSUM_ERR;
        else
          	return UPC2_NORMAL_RETURN;

		//	break;
		// for DEBUG
		//retry_count++;
	//}
    // for DEBUG
    //frame_count ++;

	//if (k == READ_DATA_MAX_TRIES)
	//	return UPC2_CKSUM_ERR;
	//else
	//	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// SendCommandEx -- calls IsConnected and IsAwaiting (which gets the SDRAM memory map and command
//                   buffer) then calls SendCommand to setup and send command
// parameters:
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
//  command				-- long containing the command code
//
//  retry_mult			-- multiple of BUSY_TEST_TRIES to use
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//			  UPC2_DSP_COMMAND_NG	if DSP unable to process command			
//
long SendCommandEx(long card_ndx, U32 command, long retry_mult)
{
	long ret_val;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	ret_val = SendCommand(card_ndx, command, retry_mult);
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// SendCommand -- sets up and sends a command to the DSP.
//
// parameters:
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
//  command				-- long containing the command code
//
//  retry_mult			-- multiple of BUSY_TEST_TRIES to use
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//
long SendCommand(long card_ndx, U32 command, long retry_mult)
{
	long ret_val;
	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		// Setup command
		SetCommandBuffer(command, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < retry_mult*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == retry_mult*BUSY_TEST_TRIES)
			continue;

		// Retry if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
			continue;

		// Return if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
			return UPC2_NORMAL_RETURN;
	}
	return UPC2_EXCEEDED_CMD_RETRY_LIMIT;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetInventory -- gets the number of PCI cards and their serial numbers
// 
// parameters:
//
//  psn 		-- pointer to an array to contain up to MAX_PCI serial numbers
//
// Returns 	-- negative if an error occurs
//			  		UPC2_COMM_ERR 		if communication failure
//			      UPC2_INVALID_INDEX 	if no UPC card with the specified index
//
//           or
//
//		   	-- number of UPC cards in the system
//
DllExport long __stdcall UPC2_PCI_GetInventory(long * psn)
{
	long    ret_val;
	long    i;

	// Get the number of cards in the system
	if ((ret_val = SelectPCI(&UPC2_Device[0], 0)) < 0)
		return UPC2_INVALID_INDEX;

	nPCI_cards = ret_val;

	// Get their serial numbers
	for (i = 0; i < nPCI_cards; i++)
	{
		// Connect to i-th
		if ((ret_val = UPC2_PCI_Connect(i)) < 0)
			return ret_val;

        // Stop data collection
        if ((ret_val = UPC2_PCI_StopDataCollection(i)) < 0)
            return ret_val;

		// Get i-th serial number
		if ((ret_val = UPC2_PCI_ReadSystemSerialNumber(i, &UPC2_SysSerNum[i])) < 0)
			return ret_val;

		*psn++ = UPC2_SysSerNum[i];
	}
	return nPCI_cards;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void LogError(LPCTSTR strErrorMsg)
{
#ifdef LOG_ERROR
	//BOOL bResult;
	//LPCTSTR ErrorString[1];
	//HANDLE hEventLog = RegisterEventSource(NULL, NULL);
	//if(!hEventLog) 
	//	return FALSE;
	//ErrorString[0] = strErrorMsg;
	//bResult = ReportEvent(hEventLog, EVENTLOG_ERROR_TYPE, 1, NULL, NULL, 1, 0, ErrorString, 0);
	//DeregisterEventSource(hEventLog);
	//return bResult;

    FILE *handle;
    
	// Open log file 
	handle = fopen("LogFile.txt", "a");

	// Write string
	fputs(strErrorMsg, handle);
	fclose(handle);
#endif
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_Connect -- attempts to connect to the n-th (zero-based_ UPC card
// 
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//		   -- positive if connected
//
DllExport long __stdcall UPC2_PCI_Connect(long card_ndx)
{
	long ret_val;
	char   str[100];

	QueryPerformanceFrequency(&frq);
	// sprintf(str, "Connect: PerfCounterFreq = %I64d Hz", frq.QuadPart);
//  OutputDebugString(str);

#ifdef LOG_ERROR
	// Log entry to function including card_ndx
	sprintf(str,"Entering UPC2_PCI_Connect with card_ndx = %d\n",card_ndx);
	LogError(str);

	// Log connection status
	sprintf(str,"IsConnected returns = %d\n",IsConnected(card_ndx));
	LogError(str);
#endif
	if ((ret_val = IsConnected(card_ndx)) != UPC2_NO_CONNECTION)
		return ret_val;

#ifdef LOG_ERROR
	// Log calling SelectPCI
	sprintf(str,"Calling SelectPCI\n");
	LogError(str);
#endif


	// Select n-th PLX PCI device
	if (SelectPCI(&UPC2_Device[card_ndx], card_ndx) < 0)
	{
#ifdef LOG_ERROR
		sprintf(str,"SelectPCI unsuccessful");
        LogError(str);
#endif
		return UPC2_INVALID_INDEX;
	}

#ifdef LOG_ERROR
	// Log SelectPCI success
	sprintf(str,"SelectPCI successful. Calling OpenPCI\n");
    LogError(str);
#endif


	// Open n-th PLX PCI device
	if (OpenPCI(&UPC2_Device[card_ndx], &UPC2_hDevice[card_ndx]) < 0)
		return UPC2_INVALID_INDEX;

#ifdef LOG_ERROR
	// Log OpenPCI success
	sprintf(str,"OpenPCI successful. Calling MapBAR\n");
    LogError(str);
#endif


	// Map BAR (sets up HPIC)
	if (Map_BAR(card_ndx) < 0)
		return UPC2_COMM_ERR;

#ifdef LOG_ERROR
	// Log MapBAR success
    sprintf(str,"MapBAR successful\n");
    LogError(str);
#endif


	// Set state to connected
	UPC2_PCI_State[card_ndx] |= UPC2_CONNECTED;

#ifdef LOG_ERROR
	// Log Connect success
	sprintf(str,"Connect successful\n");
    LogError(str);
#endif

	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_Disconnect -- attempts to disconnect ot the n-th (zero-based_ UPC card
// 
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
//			  UPC2_NO_CONNECTION 	if specified index is not connected
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//
DllExport long __stdcall UPC2_PCI_Disconnect(long card_ndx)
{
	long ret_val;

	// Check for connected 
	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	// Unmap BAR
	UnMap_BAR(card_ndx);


	// Close n-th PLX PCI device
	if (ClosePCI(UPC2_hDevice[card_ndx]) < 0)
		return UPC2_INVALID_INDEX;

	// Set state to not connected
	UPC2_PCI_State[card_ndx] &=  ~UPC2_CONNECTED;
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_Reset -- performs a reset of the PLX chip
// 
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
//			  UPC2_NO_CONNECTION 	if specified index is not connected
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//
DllExport long __stdcall UPC2_PCI_Reset(long card_ndx)
{
	long    ret_val;
    U32     Va = UPC2_Va[card_ndx]; 

	// Check for connected 
	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	// Reset n-th PLX PCI device
	PlxPciBoardReset(UPC2_hDevice[card_ndx]);

	// Delay ~ one second
	Sleep(1000);


	// Setup HPIC
	*(U32*)(Va) = 0x00010001;	  // = HWOB = 1 => first halfword is least significant


	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_SetTimestamp -- sets timestamp on all connected cards
// 
// parameters:
//
//  timestamp -- long representing the number of A/D conversion
//
// Returns --  UPC2_NO_CONNECTION if no cards connected
//
DllExport long __stdcall UPC2_PCI_SetTimestamp(long timestamp)
{
	long     ret_val;
	long		i;
	long     ts[3];
	long     ds[3];

	LARGE_INTEGER callt, calct, t0, t1;
	double       fdelta;
	//double       dtm;
	
	ds[0] = 0;				// to make compiler happy
	ts[0] = ts[1] = timestamp;
	ts[2] = -1;

	// calculate QueryPerformanceCounter call overhead
	QueryPerformanceCounter(&t0);
	QueryPerformanceCounter(&t1);
	callt.QuadPart = t1.QuadPart - t0.QuadPart;

	// calculate calc overhead
	calct.QuadPart = 0;
	QueryPerformanceCounter(&t0);
	fdelta = (double) ((t1.QuadPart - t0.QuadPart - callt.QuadPart + calct.QuadPart) / (double)frq.QuadPart);
	// simulated calc folows
	ds[0] += (long)(fdelta * 10e5);  
	ds[1] = ds[0];
	QueryPerformanceCounter(&t1);
	calct.QuadPart = t1.QuadPart - t0.QuadPart;

	//QueryPerformanceCounter(&t0);
	ret_val = UPC2_NO_CONNECTION;

	for (i = 0; i < MAX_PCI_CARDS; i++)
	{
		QueryPerformanceCounter(&t0);

		// Check for connected 
		if (UPC2_PCI_State[i] & UPC2_CONNECTED)
		{
			WriteToLocalAddressSpace(i, ts, TIMESTAMP_ADDR, sizeof(ts));
			ret_val = UPC2_NORMAL_RETURN;

			QueryPerformanceCounter(&t1);
			fdelta = (double) ((t1.QuadPart - t0.QuadPart - callt.QuadPart + calct.QuadPart) / (double)frq.QuadPart);
			ts[0] += (long)(fdelta * 10e5);
			ts[1] = ts[0];
		}
	}

	//QueryPerformanceCounter(&t1);
	//tm.QuadPart = t1.QuadPart - t0.QuadPart - callt.QuadPart;
	//dtm= (double)tm.QuadPart/(double)frq.QuadPart;

	return ret_val;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_UploadConfig -- writes the UPC2_Config structure to SDRAM
//                           and sets SDRAM memory map offset entry
//
// parameters:
//
// card_ndx 	-- long 0, 1, 2, .. representing the card's index
// pUPC2_Config -- pointer to a UPC2_Config_t  struct
//
// Returns -- negative if an error occurs. 
//            UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION 	if not connected
//
DllExport long __stdcall UPC2_PCI_UploadConfig(long card_ndx, UPC2_Config_t * pUPC2_Config)
{
	long     i,ret_val;
	U32      addr = UPC2_CONFIG_STRUCT_ADDR;

	if ((ret_val = IsConnected(card_ndx)) < 0)
	{
		// Not connected. Set demo_config
		demo_config[card_ndx].nItems = pUPC2_Config->nItems;
		demo_config[card_ndx].nSbits = pUPC2_Config->nSbits;
		demo_config[card_ndx].frame_no = 0;
		demo_config[card_ndx].scan_interval = pUPC2_Config->scan_interval;
		for (i = 0; i < pUPC2_Config->nItems; i++)
		{
			demo_config[card_ndx].scale[i] = pUPC2_Config->item[i].scale_factor; 
			demo_config[card_ndx].offset[i] = pUPC2_Config->item[i].offset; 
		}
		return ret_val;
	}

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// If returning raw data verify even number of sbits
	if ((pUPC2_Config->op_flags & 0x000000ff) == 0x00000052)
	{
		if ((pUPC2_Config->nSbits & 1) == 1)
			return UPC2_ODD_NUMBER_OF_SBITS;
	}
	// Copy UPC2_Config to SDRAM
	WriteToLocalAddressSpace(card_ndx, pUPC2_Config, 
									 UPC2_CONFIG_STRUCT_ADDR, sizeof(UPC2_Config_t));
	// Set SDRAM Memory map entry 
	WriteToLocalAddressSpace(card_ndx, &addr, 
									 UPC2_CONFIG_STRUCT_MM_ADDR, sizeof(U32));
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_UploadConfigFromPath -- writes the UPC2_Config structure to SDRAM
//                           and sets SDRAM memory map offset entry
//
// parameters:
//
// card_ndx 		-- long 0, 1, 2, .. representing the card's index
// pUPC2_ConfigPath -- pointer to a .cfg file path
//
// Returns -- negative if an error occurs. 
//            UPC2_COMM_ERR 			if communication failure
//			  UPC2_BUSY 				if DSP busy 
//			  UPC2_INVALID_INDEX 		if no UPC card with the specified index
//			  UPC2_NO_CONNECTION 		if not connected
//  		  UPC2_FILE_OPEN_ERR    	if unable to open the file
//  		  UPC2_CIONFIG_BNOT_FOUND  	if config not found
//
DllExport long __stdcall UPC2_PCI_UploadConfigFromPath(long card_ndx, char * pFilePath)
{
	FILE     *handle;
	size_t   numread;
	long     ret_val = 0;
	long     i;
	U32      addr = UPC2_CONFIG_STRUCT_ADDR;

	// Open file

	if ((handle = fopen(pFilePath, "rb")) == NULL)
	{
		OutputDebugString("Unable to open file");
		ret_val = UPC2_FILE_OPEN_ERR;
		return ret_val;
	}

	numread = fread((void *)&UPC2_Config, sizeof(UPC2_Config_t), 1, handle);
	if (numread != 1)
	{
		ret_val = UPC2_CONFIG_NOT_FOUND;
		return ret_val;
	}

	if ((ret_val = IsConnected(card_ndx)) < 0)
	{
		// Not connected. Set demo_config
		demo_config[card_ndx].nItems = UPC2_Config.nItems;
		demo_config[card_ndx].nSbits = UPC2_Config.nSbits;
		demo_config[card_ndx].frame_no = 0;
		demo_config[card_ndx].scan_interval = UPC2_Config.scan_interval;
		for (i = 0; i < UPC2_Config.nItems; i++)
		{
			demo_config[card_ndx].scale[i] = UPC2_Config.item[i].scale_factor; 
			demo_config[card_ndx].offset[i] = UPC2_Config.item[i].offset; 
		}
		return ret_val;
	}

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// If returning raw data verify even number of sbits
	if ((UPC2_Config.op_flags & 0x000000ff) == 0x00000052)
	{
		if ((UPC2_Config.nSbits & 1) == 1)
			return UPC2_ODD_NUMBER_OF_SBITS;
	}


	// Copy UPC2_Config to SDRAM
	WriteToLocalAddressSpace(card_ndx, &UPC2_Config, 
									 UPC2_CONFIG_STRUCT_ADDR, sizeof(UPC2_Config_t));
	// Set SDRAM Memory map entry 
	WriteToLocalAddressSpace(card_ndx, &addr, 
									 UPC2_CONFIG_STRUCT_MM_ADDR, sizeof(U32));
	return UPC2_NORMAL_RETURN;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_StartDataCollection -- sends a command to initiate data collection
//
// parameters:
//
// card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
//            UPC2_COMM_ERR 		if communication failure
//            UPC2_NO_CONFIG 		if UPC2_Config not loaded
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_DSP_COMMAND_NG	if DSP unable to process command			
//			  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_StartDataCollection(long card_ndx)
{
	long ret_val;
	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;

	DWORD tk, tk0, tk1;
	LARGE_INTEGER t0, t1, tm;
	double       dtm;

	QueryPerformanceCounter(&t0);

	tk0 = GetTickCount();
	if ((ret_val = IsConnected(card_ndx)) < 0)
	{
		// Not connected -- go to DEMO mode
		demo_config[card_ndx].time_in_ms = GetTickCount();
		UPC2_DSP_State[card_ndx] |= DSP_DATA_COLLECTION_STARTED;
		return ret_val;
	}

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
	{
		if (ret_val == UPC2_BUSY)
		{
			UPC2_DSP_State[card_ndx] |= DSP_DATA_COLLECTION_STARTED;
			return UPC2_NORMAL_RETURN;
		}
		else
			return ret_val;
	}


	// Verify UPC2_Config loaded
	if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
		return UPC2_NO_CONFIG;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_START_DATA_COLLECTION, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not New Command
		for (j = 0; j < 1000*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_NEW_COMMAND) == 0)
				break;
		}
		if (j == 1000*BUSY_TEST_TRIES)
			continue;

		// Retry if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
			continue;

		// Wait for collecting data
		for (j = 0; j < 1000*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			// Return if DSP collecting data
			if (cmd_status & UPC2_DSP_COLLECTING_DATA)
			{


				tk1 = GetTickCount();
				tk = tk1 - tk0;
				QueryPerformanceCounter(&t1);
				tm.QuadPart = t1.QuadPart-t0.QuadPart;
				dtm= (double)tm.QuadPart/(double)frq.QuadPart;

				UPC2_DSP_State[card_ndx] |= DSP_DATA_COLLECTION_STARTED;
				return UPC2_NORMAL_RETURN;
			}
		}

		if (j == 1000*BUSY_TEST_TRIES)
			continue;

	}
	return UPC2_EXCEEDED_CMD_RETRY_LIMIT;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_SetSFandOffset -- Sets the scale factor and offset for item n
//
// parameters:
//
//  card_ndx      -- long 0, 1, 2, .. representing the card's index
//  scale_factor  -- FP value
//  offset        -- FP value
//  item          -- long item number
//
// Returns 		-- negative if an error occurs. 
// 			         UPC2_COMM_ERR 			    if communication failure
//                   UPC2_NO_CONFIG 			if UPC2_Config not loaded
//			         UPC2_INVALID_INDEX   		if no UPC card with the specified index
//			         UPC2_NO_CONNECTION	   	    if not connected
//                   UPC2_INVALID_ITEM          if item number non existent
//
//
DllExport long __stdcall UPC2_PCI_SetSFandOffset(long card_ndx, float scale_factor, float offset, long item)
{
   long  ret_val;
   Uint32 sf_offset;
   Uint32 offset_offset;
   

   if ((ret_val = IsConnected(card_ndx)) < 0)
      return ret_val;

   // Verify UPC2_Config loaded
   if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
      return UPC2_NO_CONFIG;

   // Copy UPC2_Config to DLL's buffer
   ret_val = ReadFromLocalAddressSpace(card_ndx, UPC2_CONFIG_STRUCT_ADDR,
                                            &UPC2_Config, sizeof(UPC2_Config_t));
   if (ret_val < 0)
      return ret_val;

   // Get number of items
   ret_val = UPC2_Config.nItems;
   if ((item >= ret_val) || (item < 0))
      return UPC2_INVALID_ITEM;
    
    
   // Get offset to scale factor
   sf_offset = (Uint32)&UPC2_Config.item[item].scale_factor - (Uint32)&UPC2_Config;

   // Get offset to offset
   offset_offset = (Uint32)&UPC2_Config.item[item].offset - (Uint32)&UPC2_Config;

   // write new offset
   WriteToLocalAddressSpace(card_ndx, &offset, 
                                UPC2_CONFIG_STRUCT_ADDR + offset_offset, sizeof(offset));

   // write new scale factor
   WriteToLocalAddressSpace(card_ndx, &scale_factor, 
                                UPC2_CONFIG_STRUCT_ADDR + sf_offset, sizeof(scale_factor));


   ret_val = UPC2_NORMAL_RETURN;
   return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_SetStartFrame -- Sets the starting frame for subsequent GetData (UPC2_FROM_START_FRAME)
//
// parameters:
//
//  card_ndx    -- long 0, 1, 2, .. representing the card's index
//
// Returns 		-- negative if an error occurs. 
//			         UPC2_INVALID_INDEX   		 if no UPC card with the specified index
//			         UPC2_NO_CONNECTION	   	     if not connected
//
DllExport long __stdcall UPC2_PCI_SetStartFrame(long card_ndx)
{
	long ret_val;
	UPC2_ConvertedDataFramePoolHdr_t FrameHdrImage;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	// Read the pool header
	ReadFromLocalAddressSpace(card_ndx, CONVERTED_DATA_FRAMES_POOL_HDR_ADDR,
									  &FrameHdrImage, sizeof(FrameHdrImage));
	// Update pointer in the header
	FrameHdrImage.pStart = FrameHdrImage.pNew;
	WriteToLocalAddressSpace(card_ndx, &FrameHdrImage.pNew,
									 CONVERTED_DATA_FRAMES_POOL_HDR_ADDR + PSTART_OFFSET, sizeof(U32));

	ret_val = UPC2_NORMAL_RETURN;
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void CheckFrameNo(void * pframe, long nFrames, U32 frm_size)
{
	long    i, j;
	//long    frame_no;
	void *  cframe = pframe;

	//frame_no = ((UPC2_ConvertedDataFrame_t *) cframe)->frame_no;
	for (i = 0; i < nFrames; i++)
	{
		if (((UPC2_ConvertedDataFrame_t *) cframe)->frame_no != frame_no++)
			j = 0;
		(Uint32)cframe += frm_size;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// CheckFrame -- Returns the difference between the computed checkword and the stored checkword
//
//
U32 CheckFrame(U32 frm_size, void * pFrame)
{
	U32   cw;
	U32   ret_val = 0;
#if 0
	//cw = Calculate32BitChecksum((frm_size - 4)/4, (U32 *) pFrame);
	cw = Calculate32BitCRC(frm_size - 4, (U8 *) pFrame);
	ret_val = *(U32 *)((U32)pFrame + frm_size - 4) - cw;
	if (ret_val)
	{
		if (((UPC2_ConvertedDataFrame_t *) pFrame)->frame_no == frame_no)
			cw = 2;
	}
	else
		frame_no++;
#endif
    if (((UPC2_ConvertedDataFrame_t *) pFrame)->frame_no == frame_no)
		frame_no++;
    else
        cw = 2;
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetUnreadFrameCount	- reads count of unread frames in the buffer
//
DllExport long __stdcall UPC2_PCI_GetUnreadFrameCount(long card_ndx)
{
	UPC2_ConvertedDataFramePoolHdr_t FrameHdrImage;
	U32				FrameAddr;		// local address of converted frame
	U32				frm_size;
	long			   nFramesUnread;

    // Read the pool header
	ReadFromLocalAddressSpace(card_ndx, CONVERTED_DATA_FRAMES_POOL_HDR_ADDR,
									  &FrameHdrImage, sizeof(FrameHdrImage));

	frm_size = FrameHdrImage.FrameSize;   // includes check word
	FrameAddr = (U32)FrameHdrImage.pStart;

	// Determine nFramesUnread
	if (FrameHdrImage.pNew < FrameHdrImage.pStart)
		nFramesUnread = FrameHdrImage.MaxFrames - ((U32)FrameHdrImage.pStart - (U32)FrameHdrImage.pNew) / frm_size;
	else
		nFramesUnread = ((U32)FrameHdrImage.pNew - (U32)FrameHdrImage.pStart) / frm_size;
    
	return nFramesUnread;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetData -- reads converted data from the ring buffer and updates the converted data
//					   frame header and the frame status word.
//
//					  If access method is UPC2_NEWEST_DATA a single frame is read
//
//					  If access method is UPC2_NO_GAPS or UPC2_START_FROM_FRAME N, frames are 
//                       read where  N = min(nFrames , number of frames unread) and the number
//                       read is returned to the caller
//
//					  If UPC2_NO_GAPS is specified the frames are transferred directly
//                       from DSP memory to the caller. Otherwise the data is read a 
//                       a frame at a time assuming the caller has specified 24 items/frame
//
//					   	 Additionally, the StartFrame pointer is advanced to the next
//                   	   available frame.
//
// parameters:
//
//  card_ndx    -- long 0, 1, 2, .. representing the card's index
//
//  access_type -- UPC2_NO_GAPS, UPC2_FROM_START_FRAME, UPC2_NEWEST_DATA or UPC2_FROM_LOAD_PTR
//
//	 nFrames 	-- long specifying the number of frames to read (not used if NEWEST)
//
//  pFrame 		-- pointer to a destination buffer for the converted data frame(s)
//
// Returns 		-- negative if an error occurs. 
//			         UPC2_INVALID_INDEX   		 		if no UPC card with the specified index
//			         UPC2_NO_CONNECTION	  	     		if not connected
//                   UPC2_DATA_COLLECTION_NOT_STARTED	if data not being collected
//
//         		-- number of frames read if no error
//
DllExport long __stdcall UPC2_PCI_GetData(long card_ndx, long access_type, long nFrames, void * pFrame)
{
	UPC2_ConvertedDataFramePoolHdr_t FrameHdrImage;
	UPC2_ConvertedDataFrame_t * pF = (UPC2_ConvertedDataFrame_t *)pFrame;
	long			ret_val;
	U32				FrameAddr;		// local address of converted frame
	long			nFramesToEnd, pFrames;
	long			nFramesUnread;
	long			i, j, nItems, fcnt, fread;
	U32			    t0, t1, t; 
	float			val;
	U32			    pFrame0 = (U32) pFrame;
	long			jmax = 3;
	static  long    retry_count = 0;
	U32				frm_size, frm_incr;

	if ((UPC2_DSP_State[card_ndx] & DSP_DATA_COLLECTION_STARTED) == 0)
	{
		return UPC2_DATA_COLLECTION_NOT_STARTED;
	}

	if ((ret_val = IsConnected(card_ndx)) < 0)
	{
		////////////////////////////////////////////////////////////////////////////////   
		//    	D E M O    M O D E
		////////////////////////////////////////////////////////////////////////////////   

		// Not connected. Send data based on demo_config
		nItems = demo_config[card_ndx].nItems;
		if (access_type == UPC2_FROM_START_FRAME)
		{
			t0 = demo_config[card_ndx].time_in_ms;
			t1 = GetTickCount();
			demo_config[card_ndx].time_in_ms = t1;

			if (t1 > t0)
				t = t1 - t0;
			else
				t = 0xffffffff - t1 + t0;

			// frames read = time in usec / (number of frames * frame scan time in usecs)
			//             = (time in mscec * 1000) / (number of frames * nSbits * 10)   
			//             = (time in mscec * 100) / (number of frames * nSbits)   

			fread = (100 * t) / (demo_config[card_ndx].scan_interval * demo_config[card_ndx].nSbits); 
			fcnt = (nFrames > fread) ? fread : nFrames;
		}
		else
			fcnt = 1;

		for (j = 0; j < fcnt; j++)
		{
			pF->frame_no = demo_config[card_ndx].frame_no++;
			val = (pF->frame_no % 10) * 0.1f;  // sawtooth that goes 0 to 1 in 10 steps
			pF->timestamp = pF->frame_no * demo_config[card_ndx].scan_interval * demo_config[card_ndx].nSbits;
			for (i = 0; i < nItems; i++)
			{
				//  Random number returned by the following
				//   pF->data[i] = (i + 1 + rand()) * demo_config[card_ndx].scale[i] + demo_config[card_ndx].offset[i];
				// Sawtooth function returned by the following
				pF->data[i] = (i + 1 + val) * demo_config[card_ndx].scale[i] + demo_config[card_ndx].offset[i];
			}
			// Bump to next frame
			//pF = (UPC2_ConvertedDataFrame_t *)((Uint32)(pF) + demo_config[card_ndx].nItems * 4 + 8);
			// 4-21-05 for the time being always assume data aray has room for 24 items ( = 4*24 + 8 = 104)
			pF = (UPC2_ConvertedDataFrame_t *)((Uint32)(pF) + 104);
		}
		return fcnt;
	}
	// Test for Data started

	////////////////////////////////////////////////////////////////////////////////   
	//    	R E A L   D A T A   M O D E
	////////////////////////////////////////////////////////////////////////////////   

	// Read the pool header
	ReadFromLocalAddressSpace(card_ndx, CONVERTED_DATA_FRAMES_POOL_HDR_ADDR,
									  &FrameHdrImage, sizeof(FrameHdrImage));

	frm_size = FrameHdrImage.FrameSize;   // includes check word
	
	if (access_type == UPC2_NO_GAPS)
		frm_incr =  frm_size - 4;
	else
		// EaesySense array has room for 24 items ( = 4*24 + 8 = 104)
		frm_incr = EZ_SENSE_FRAME_SIZE;


	if (access_type == UPC2_FROM_LOAD_PTR)
    {
		//////////////////////////////////////////////////////
		// Process Read from Load Pointer request
		//////////////////////////////////////////////////////
		FrameAddr = (U32)FrameHdrImage.pLoad;

		// Read the data frame(s)
		ret_val = ReadFromLocalAddressSpace(card_ndx, FrameAddr, pFrame, FrameHdrImage.FrameSize);
        if (ret_val < 0)
            return ret_val;
        nFrames = 1;
    }
    else if (access_type == UPC2_NEWEST_DATA)
    {
		//////////////////////////////////////////////////////
		// Process NEWEST request
		//////////////////////////////////////////////////////
		FrameAddr = (U32)FrameHdrImage.pNew;

		// Read the data frame(s)
		ret_val = ReadFromLocalAddressSpace(card_ndx, FrameAddr, pFrame, FrameHdrImage.FrameSize);
        if (ret_val < 0)
            return ret_val;
		nFrames = 1;
    }
    else
    {
		//////////////////////////////////////////////////////
		// Process START_FROM_FRAME request
		//////////////////////////////////////////////////////

		FrameAddr = (U32)FrameHdrImage.pStart;

		// Determine nFramesUnread
		if (FrameHdrImage.pNew < FrameHdrImage.pStart)
			nFramesUnread = FrameHdrImage.MaxFrames - ((U32)FrameHdrImage.pStart - (U32)FrameHdrImage.pNew) / frm_size;
		else
			nFramesUnread = ((U32)FrameHdrImage.pNew - (U32)FrameHdrImage.pStart) / frm_size;

		if (nFramesUnread == 0)
			return 0;

		// Reset nFrames if request too large 
		if (nFrames > nFramesUnread)
			nFrames = nFramesUnread -1;

		// Determine nFramesToEnd
		nFramesToEnd = ((U32)FrameHdrImage.pLast - (U32)FrameHdrImage.pStart)/frm_size + 1;

		// Read the data frame(s)
		if (nFrames < nFramesToEnd)
		{
			//////////////////////////////////////////////////////
			// request does not wrap 
			//////////////////////////////////////////////////////
			for (i = 0; i < nFrames; i++)
			{
				ret_val = ReadWithCheckFromLocalAddressSpace(card_ndx, FrameAddr, pFrame, frm_size);
				if (ret_val < 0)
					return ret_val;
                //CheckFrame(frm_size, pFrame);
				FrameAddr += frm_size;
				(U32) pFrame += frm_incr;
			}
			newFrameAddr = (U32)FrameHdrImage.pStart + (U32)frm_size * nFrames;
		}
		else
		{
			//////////////////////////////////////////////////////
			// request wraps -- read to end
			//////////////////////////////////////////////////////
			for (i = 0; i < nFramesToEnd; i++)
			{
				ret_val = ReadWithCheckFromLocalAddressSpace(card_ndx, FrameAddr, pFrame, frm_size);
				if (ret_val < 0)
					return ret_val;
                //CheckFrame(frm_size, pFrame);
				FrameAddr += frm_size;
				(U32) pFrame += frm_incr;
			}

			//////////////////////////////////////////////////////
			// then read balance from start of buffer
			//////////////////////////////////////////////////////
			FrameAddr = (U32)FrameHdrImage.pFrame1;
			pFrames = nFrames - nFramesToEnd; 
			for (i = 0; i < pFrames; i++)
			{
				ret_val = ReadWithCheckFromLocalAddressSpace(card_ndx, FrameAddr, pFrame, frm_size);
				if (ret_val < 0)
					return ret_val;
                //CheckFrame(frm_size, pFrame);
				FrameAddr += frm_size;
				(U32) pFrame += frm_incr;
			}
			newFrameAddr = (U32)FrameHdrImage.pFrame1 + (U32)frm_size * pFrames;
		}
		// Update pointer in the header
		WriteToLocalAddressSpace(card_ndx, &newFrameAddr,
										 CONVERTED_DATA_FRAMES_POOL_HDR_ADDR + PSTART_OFFSET, sizeof(U32));
	}
	return nFrames;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//

// UPC2_PCI_StopDataCollection -- sends a command to terminate data collection.
// 
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs. 
//            UPC2_COMM_ERR 			  if communication failure
//			  UPC2_INVALID_INDEX 		  if no UPC card with the specified index
//			  UPC2_DSP_COMMAND_NG		  if DSP unable to process command			
//			  UPC2_NO_CONNECTION		  if not connected
//
DllExport long __stdcall UPC2_PCI_StopDataCollection(long card_ndx)
{
	long ret_val;
	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;

	DWORD tk, tk0, tk1;
	LARGE_INTEGER t0, t1, tm;
	double       dtm;

	QueryPerformanceCounter(&t0);

	tk0 = GetTickCount();

	if ((ret_val = IsConnected(card_ndx)) < 0)
	{
		return ret_val;
	}

//#ifdef USE_SEND_COMMAND
	ret_val = SendCommand(card_ndx, UPC2_DSP_STOP_DATA_COLLECTION, 100);
//#else
	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_STOP_DATA_COLLECTION, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;

			tk1 = GetTickCount();
			tk = tk1 - tk0;
			QueryPerformanceCounter(&t1);
			tm.QuadPart = t1.QuadPart-t0.QuadPart;
			dtm= (double)tm.QuadPart/(double)frq.QuadPart;

			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

//#endif
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_SaveConfigToFlash -- sends a command to save the current configuration to Flash memory
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
// 			  UPC2_COMM_ERR 			if communication failure
//			  UPC2_BUSY 				if DSP busy 
//            UPC2_NO_CONFIG 			if UPC2_Config not loaded
//			  UPC2_INVALID_INDEX 		if no UPC card with the specified index
//			  UPC2_DSP_COMMAND_NG	    if DSP unable to process command			
//			  UPC2_NO_CONNECTION	    if not connected
//
DllExport long __stdcall UPC2_PCI_SaveConfigToFlash(long card_ndx)
{
	long ret_val;
#ifndef USE_SEND_COMMAND
	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;
#endif

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Verify UPC2_Config loaded
	if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
		return UPC2_NO_CONFIG;

#ifdef USE_SEND_COMMAND
	ret_val = SendCommand(card_ndx, UPC2_DSP_SAVE_CONFIGURATION_TO_FLASH, 2000);
#else



	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_SAVE_CONFIGURATION_TO_FLASH, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_LoadConfigFromFlash -- sends a command to load the current configuration from flash
//                                  memory into SDRAM
// parameters:
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//			  UPC2_DSP_COMMAND_NG	if DSP unable to process command			
//
DllExport long __stdcall UPC2_PCI_LoadConfigFromFlash(long card_ndx)

{
	long ret_val;

#ifdef USE_SEND_COMMAND
	ret_val = SendCommandEx(card_ndx, UPC2_DSP_LOAD_CONFIGURATION_FROM_FLASH, 100);
#else
	
   long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_LOAD_CONFIGURATION_FROM_FLASH, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	return ret_val;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetNumberOfItems -- gets the number of items from the current configuration
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
// 			  UPC2_COMM_ERR 			if communication failure
//			  UPC2_BUSY 				if DSP busy 
//            UPC2_NO_CONFIG 			if UPC2_Config not loaded
//			  UPC2_INVALID_INDEX 		if no UPC card with the specified index
//			  UPC2_NO_CONNECTION	    if not connected
//
DllExport long __stdcall UPC2_PCI_GetNumberOfItems(long card_ndx)
{
	long ret_val;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = GetMemoryMapPlus(card_ndx)) < 0)
		return ret_val;

	// Verify UPC2_Config loaded
	if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
		return UPC2_NO_CONFIG;

	// Copy UPC2_Config to DLL's buffer
	ret_val = ReadFromLocalAddressSpace(card_ndx, UPC2_CONFIG_STRUCT_ADDR,
													&UPC2_Config, sizeof(UPC2_Config_t));
	if (ret_val < 0)
		return ret_val;

	// Get number of items
	ret_val = UPC2_Config.nItems;
   return ret_val;

}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_DownloadConfig -- reads the current configuration
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
// pUPC2_Config -- pointer to a UPC2_Config_t  struct
//
// Returns -- negative if an error occurs.
// 			  UPC2_COMM_ERR 			if communication failure
//			  UPC2_BUSY 				if DSP busy 
//            UPC2_NO_CONFIG 			if UPC2_Config not loaded
//			  UPC2_INVALID_INDEX 		if no UPC card with the specified index
//			  UPC2_NO_CONNECTION	    if not connected
//
DllExport long __stdcall UPC2_PCI_DownloadConfig(long card_ndx, UPC2_Config_t * pUPC2_Config)
{
	long ret_val;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = GetMemoryMapPlus(card_ndx)) < 0)
		return ret_val;

	// Verify UPC2_Config loaded
	if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
		return UPC2_NO_CONFIG;

	// Copy UPC2_Config to user's buffer
	ret_val = ReadFromLocalAddressSpace(card_ndx, UPC2_CONFIG_STRUCT_ADDR,
													pUPC2_Config, sizeof(UPC2_Config_t));
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_InCircuitProgram -- sends a command to write the binary image to flash memory
//
// parameters:
//
//  card_ndx 	-- long 0, 1, 2, .. representing the card's index
//
//	dest		-- UPC2_DSP_BOOT or UPC2_DSP_PGM
//
//  pFilePath	-- pointer to a path for of an Intel hex file 
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		   if communication failure
//			  UPC2_BUSY 			   if DSP busy 
//			  UPC2_DSP_COMMAND_NG   if DSP unable to process command			
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION    if not connected
//			  UPC2_PROGRAM_ERR		if programming was unsuccessful (Categorize)
//
DllExport long __stdcall UPC2_PCI_InCircuitProgram(long card_ndx, long dest, char * pFilePath)
{
	long ret_val;
	long i,j;
	long  pgm_size;
	U32  CRC;
	UPC2_CommandBuffer_t     CommandBuffer;
	//  char str[80];


	// For debug
	//pSize = pInfo->code_size;
	//memcpy(buf, pFile, pSize);
	//return UPC2_NORMAL_RETURN;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	if ((pgm_size = DownloadHexFile(pFilePath)) < 0)
		return pgm_size;

	// Copy CRC of binary image to CommandDataBuffer
	CRC = Calculate32BitCRC(pgm_size, pgm_buf);
	WriteToLocalAddressSpace(card_ndx, &CRC, COMMAND_DATA_BUFFER_ADDR, sizeof(U32));

	// Copy binary image to Command Data Buffer + 4
	WriteToLocalAddressSpace(card_ndx, pgm_buf, COMMAND_DATA_BUFFER_ADDR+4, pgm_size);

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_IN_CIRCUIT_PROGRAM, &CommandBuffer);

		// Add parameters and recalculate CRC
		CommandBuffer.parameter[0] = dest;
		CommandBuffer.parameter[1] = pgm_size;
		CommandBuffer.CRC = Calculate32BitCRC(sizeof(UPC2_CommandBuffer_t) - 4, ((U8 *)&CommandBuffer) + 4);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
        //
        //  MaxG 2-26-2008 changed mult below from 8000 to 40000 (=> 4,000,000 tries)
        //                 otherwise timed out when programming sn 131665
        //
        //                 Flashing required 2,500,931
        //
		for (j = 0; j < 40000*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 40000*BUSY_TEST_TRIES)
			continue;

		// Retry if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
			continue;

		// Return if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
			return UPC2_NORMAL_RETURN;

	}
	return UPC2_EXCEEDED_CMD_RETRY_LIMIT;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_DownloadProgram -- reads the binary image of the program of the selected device
//
// parameters:
//
// card_ndx -- long 0, 1, 2, .. representing the card's index
//
// src 		-- UPC2_DSP_BOOT or UPC2_DSP_PGM
//
// pBuf		-- pointer to a buffer to contain the binary image of program
//             followed by the 32-bit CRC
//
// Returns  -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_DSP_COMMAND_NG   if DSP unable to process command			
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION    if not connected
//			  UPC2_BAD_CRC		    if downloaded image has bad CRC
//
DllExport long __stdcall UPC2_PCI_DownloadProgram(long card_ndx, long src, void * pBuf)

{
	long ret_val;
	long i,j;
	U32  CRC;
	UPC2_CommandBuffer_t     CommandBuffer;
	sw_info_t                sw_info;

	// For debug
	//memcpy(pFile, buf, pSize);
	//return UPC2_NORMAL_RETURN;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_DOWNLOAD_CALIBRATION_DATA, &CommandBuffer);

		// Add parameters and recalculate CRC
		CommandBuffer.parameter[0] = src;
		CommandBuffer.CRC = Calculate32BitCRC(sizeof(UPC2_CommandBuffer_t) - 4, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;
	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

	if (ret_val == UPC2_NORMAL_RETURN)
	{
		// Determine size of download
		ReadFromLocalAddressSpace(card_ndx, COMMAND_DATA_BUFFER_ADDR,
										  &sw_info, sizeof(sw_info));

		// Copy download from command buffer to callers area
		ReadFromLocalAddressSpace(card_ndx, COMMAND_DATA_BUFFER_ADDR,
										  pBuf, sw_info.code_size+sizeof(sw_info)+sizeof(U32));
		// Verify CRC
		CRC = Calculate32BitCRC(sw_info.code_size+sizeof(sw_info), pBuf);
		if (CRC != *((U32 *)pBuf+sw_info.code_size+sizeof(sw_info)+sizeof(U32)))
			ret_val = UPC2_BAD_CRC;
	}

	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_UploadCalibrationData -- writes the UPC2_Calibration_Data structure into the Channel
//                                        Adjustment Table in SDRAM
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
//  pUPC2_Calib_data	-- pointer to a UPC2_calib_data structure
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_INVALID_INDEX	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION    if not connected
//
DllExport long __stdcall UPC2_PCI_UploadCalibrationData(long card_ndx, UPC2_Calib_data_t * pUPC2_Calib_data)

{
	long     ret_val;
	U32      addr = UPC2_CALIB_STRUCT_TABLE_ADDR;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	if (SDRAM_Image.MemoryMap.pUPC2_Config == 0)
		return UPC2_NO_CONFIG;

	// Copy calibration data to SDRAM
	WriteToLocalAddressSpace(card_ndx, pUPC2_Calib_data,
									 UPC2_CALIB_STRUCT_TABLE_ADDR, sizeof(UPC2_Calib_data_t));
	// Set SDRAM Memory map entry 
	WriteToLocalAddressSpace(card_ndx, &addr, 
									 UPC2_CALIB_STRUCT_TABLE_MM_ADDR, sizeof(U32));
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_DownloadCalibrationData -- reads the UPC2_Calibration_Data structure from the Channel 
//                                        Adjustment Table in SDRAM
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
//  pUPC2_Calib_data	-- pointer to a UPC2_calib_data structure
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_NO_CALIB_DATA	if no Channel adjustment Table. 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//			  UPC2_NO_CONNECTION    if not connected
//
DllExport long __stdcall UPC2_PCI_DownloadCalibrationData(long card_ndx, UPC2_Calib_data_t * pUPC2_Calib_data)

{
	long ret_val;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Verify that Calibration data is loaded 
	if (SDRAM_Image.MemoryMap.pUPC2_Calib == 0)
		return UPC2_NO_CALIB_DATA;

	// Copy UPC2_Calib_data to user's buffer
	ret_val = ReadFromLocalAddressSpace(card_ndx, UPC2_CALIB_STRUCT_TABLE_ADDR,
													pUPC2_Calib_data, sizeof(UPC2_Calib_data_t));
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_SaveCalibrationDataToFlash -- sends a command to write the Channel Adjustment Table 
//                                        from SDRAM into flash memory
//
// parameters:
//
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_NO_CALIB_DATA	if no Channel adjustment Table. 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//			  UPC2_DSP_COMMAND_NG	if DSP unable to process command			
//
DllExport long __stdcall UPC2_PCI_SaveCalibrationDataToFlash(long card_ndx)
{
	long ret_val;
 
#ifdef USE_SEND_COMMAND
	ret_val = SendCommandEx(card_ndx, UPC2_DSP_SAVE_CALIBRATION_DATA_TO_FLASH, 1000);
#else
   long i,j;
   UPC2_CommandBuffer_t     CommandBuffer;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Verify calibration data loaded
	if (SDRAM_Image.MemoryMap.pUPC2_Calib == 0)
		return UPC2_NO_CALIB_DATA;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		//ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_SAVE_CALIBRATION_DATA_TO_FLASH, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));

		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			// for DEBUG don't allow the case of status not set
			if (cmd_status == 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;
	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_LoadCalibrationDataFromFlash -- sends a command to load the UPC2_Calibration_Data structure
//                                          from flash memory into the Channel Adjustment Table in SDRAM
//
// parameters:
//
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
//  pUPC2_Calib_data	-- pointer to a UPC2_Calib_dtata structure
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_NO_CALIB_DATA	if no Channel adjustment Table. 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//			  UPC2_DSP_COMMAND_NG	if DSP unable to process command			
//
DllExport long __stdcall UPC2_PCI_LoadCalibrationDataFromFlash(long card_ndx)

{
	long ret_val;
	
#ifdef USE_SEND_COMMAND
	ret_val = SendCommandEx(card_ndx, UPC2_DSP_LOAD_CALIBRATION_DATA_FROM_FLASH, 100);
#else

	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_LOAD_CALIBRATION_DATA_FROM_FLASH, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < 100*BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == 100*BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	return ret_val;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_WriteSystemSerialNumber -- writes the system serial number
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
//  sn	-- 32-bit unsigned system serial number
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_WriteSystemSerialNumber(long card_ndx, U32 sn)

{
   long    ret_val;
   long    i, j, ver;
   UPC2_CommandBuffer_t     CommandBuffer;



   // Determine version of DSP software
   ver = get_DSP_code_version(card_ndx);

   // If version less than 1.7x write serial number to PLX9030 EEPROM
   if (ver < 17)
   {
      ret_val = UPC2_PCI_WriteSystemSerialNumberToEEPROM(card_ndx, sn);
   }
   // Otherwise write serial number to flash memory
   else
   {
      if ((ret_val = IsConnected(card_ndx)) < 0)
          return ret_val;

      if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
          return ret_val;

   	  // Try n-times
	  for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	  {
	     // Setup command
		 SetCommandBuffer(UPC2_DSP_SET_SERIAL_NUMBER, &CommandBuffer);

	 	 // Add parameters and recalculate CRC
		 CommandBuffer.parameter[0] = sn;
		 CommandBuffer.CRC = Calculate32BitCRC(sizeof(UPC2_CommandBuffer_t) - 4, ((U8 *)&CommandBuffer) + 4);
   
         // Send command
		 WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		 // Wait for not busy
		 for (j = 0; j < 1000*BUSY_TEST_TRIES; j++)
		 {
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		 }
		 if (j == 1000*BUSY_TEST_TRIES)
			continue;

		 // Retry if bad CRC
		 if (cmd_status & UPC2_DSP_BAD_CRC)
			continue;

		 // Return if completed OK
		 if (cmd_status & UPC2_DSP_COMPLETED_OK)
			return UPC2_NORMAL_RETURN;
	  }
   }
   if (i == SEND_CMD_MAX_TRIES)
	   ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;
   
   return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_WriteSystemSerialNumberToEEPROM -- writes the system serial number to EEPROM  
//
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
//  sn	-- 32-bit unsigned system serial number
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_WriteSystemSerialNumberToEEPROM(long card_ndx, U32 sn)

{
	long         ret_val;
	RETURN_CODE  rc;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	// Write custom data to non-PLX used EEPROM space 
	rc = PlxVpdWrite(
						 UPC2_hDevice[card_ndx],
						 0xc0,		 // start of VPD area
						 sn 
						 ); 

	if (rc != ApiSuccess)
		return UPC2_COMM_ERR;
	else
		return UPC2_NORMAL_RETURN;     
}
/*
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// getdays -- converts mm/dd/yyyy or mm dd yyyy or mm/dd/yyyy into days
//
//
long getdays(char * strDate)
{
   long	days;
   char *  token;
   char    seps[] = "-/ ";
   int mm, dd, yy;

   token = strtok(strDate, seps);
    
   mm = atoi(token);
   token = strtok( NULL, seps);

   dd = atoi(token);
   token = strtok( NULL, seps);
	
   yy = atoi(token);
   token = strtok( NULL, seps);
   
   days = mm*30 + dd + yy*360;

   return days;

}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// get_DSP_code_version -- gets units and tenths digit of DSP software version as a whole number
//
//
long get_DSP_code_version(long card_ndx)
{
    long       ret_val;
    sw_info_t  SWinfo;
    sw_info_t  DLLinfo;
    char *     pVer;
    long       units, tenths;

	// Determine version of DSP software
    ret_val = UPC2_PCI_GetSysInfo(card_ndx, &SWinfo, &DLLinfo);
    if (ret_val == UPC2_NORMAL_RETURN)
    {
       pVer = strstr(SWinfo.version, "Production v");
       if (pVer)
       {
          units = *(pVer + 12) - 0x30;
          tenths = *(pVer + 14) - 0x30;
          return  units*10 + tenths;
       }
    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_ReadSystemSerialNumber -- reads the system serial number 
//
// parameters:
//
//
//  card_ndx	-- long 0, 1, 2, .. representing the card's index
//
//  psn	-- pointer to 32-bit unsigned system serial number
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_ReadSystemSerialNumber(long card_ndx, U32 *psn)

{
    long    ret_val  = UPC2_NORMAL_RETURN;
    long    sn, ver;

    // Initialize returned sn to zero 
    *psn = 0;

	// Determine version of DSP software
    ver = get_DSP_code_version(card_ndx);

    // If version 1.7x or later get serial number from flash memory
    if (ver > 16)
    {
      ret_val = SendCommandEx(card_ndx, UPC2_DSP_GET_SERIAL_NUMBER, 1);
      if (ret_val == UPC2_NORMAL_RETURN)
      {
         // Copy from command buffer to callers area
         ReadFromLocalAddressSpace(card_ndx, COMMAND_DATA_BUFFER_ADDR,
                                      psn, sizeof(psn));
      }
    }
    // Otherwise get serial number from PLX9030 EEPROM
    else
    {
       ret_val = UPC2_PCI_ReadSystemSerialNumberFromEEPROM(card_ndx, &sn);
       if (ret_val == UPC2_NORMAL_RETURN)
          *psn = sn;
    }
	return ret_val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_ReadSystemSerialNumberFromEEPROM -- reads the system serial number from EEPROM  
//
// parameters:
//
//
//  card_ndx	-- long 0, 1, 2, .. representing the card's index
//
//  psn	-- pointer to 32-bit unsigned system serial number
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_ReadSystemSerialNumberFromEEPROM(long card_ndx, U32 *psn)

{
	long         ret_val;
	RETURN_CODE  rc;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	*psn = PlxVpdRead(
						  UPC2_hDevice[card_ndx],
						  0xc0,		  // start of VPD area
						  &rc
						  ); 

	if (rc != ApiSuccess)
		return UPC2_COMM_ERR;
	else
		return UPC2_NORMAL_RETURN;     
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetSysInfo -- gets system information specifically version, creation date and code size 
//                        (in bytes) for the DSP and this DLL
//
// parameters:
//
//  card_ndx 	  -- long 0, 1, 2, .. representing the card's index
//
//	pSWinfo -- pointer to a buffer for sw_info struct
//
//	pDLLinfo -- pointer to a buffer for sw_info struct
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_NO_SYS_INFO		if UPC2_SysInfo not available
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_GetSysInfo(long card_ndx, sw_info_t * pSWinfo,
															sw_info_t * pDLLinfo)
{
	long ret_val;
#ifndef USE_SEND_COMMAND
	long i,j;
	UPC2_CommandBuffer_t     CommandBuffer;
#endif	
   sw_info_t    DLLinfo =
	{
		"Prod 1.0.0.21",
		__DATE__,
		__TIME__,
		81920				  // size of DLL in bytes
	};

#ifdef USE_SEND_COMMAND
	ret_val = SendCommandEx(card_ndx, UPC2_DSP_GET_SYSTEM_INFO, 1);
#else

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_GET_SYSTEM_INFO, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	if (ret_val == UPC2_NORMAL_RETURN)
	{
		// Copy UPC2_SysInfo from command buffer to callers area
		ReadFromLocalAddressSpace(card_ndx, COMMAND_DATA_BUFFER_ADDR,
										  pSWinfo, sizeof(sw_info_t));

		memcpy(pDLLinfo, &DLLinfo, sizeof(sw_info_t));
	}
	return ret_val;

}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_GetSysOpInfo -- gets system information specifically the local address of
//                             (1) SbitTbl;
//                             (2) init_oldch_data;
//                             (3) int_count;
//                             (4) filter_div.
// parameters:
//
//  card_ndx -- long 0, 1, 2, .. representing the card's index
//
//	pOPinfo -- pointer to a buffer for op_info struct
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		if communication failure
//			  UPC2_BUSY 			if DSP busy 
//			  UPC2_NO_SYS_INFO		if UPC2_SysInfo not available
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//      	  UPC2_NO_CONNECTION	if not connected
//
DllExport long __stdcall UPC2_PCI_GetSysOpInfo(long card_ndx, op_info_t * pOPinfo)
{
	long ret_val;

#ifdef USE_SEND_COMMAND
	ret_val = SendCommandEx(card_ndx, UPC2_DSP_GET_SYSTEM_OP_INFO, 1);
#else

	long i,j;
	UPC2_CommandBuffer_t    CommandBuffer;

	if ((ret_val = IsConnected(card_ndx)) < 0)
		return ret_val;

	if ((ret_val = IsAwaitingCommand(card_ndx)) < 0)
		return ret_val;

	// Try n-times
	for (i=0; i < SEND_CMD_MAX_TRIES; i++)
	{
		ret_val = 0;

		// Setup command
		SetCommandBuffer(UPC2_DSP_GET_SYSTEM_OP_INFO, &CommandBuffer);

		// Send command
		WriteToLocalAddressSpace(card_ndx, &CommandBuffer, 
										 COMMAND_BUFFER_ADDR, sizeof(CommandBuffer));
		// Wait for not busy
		for (j = 0; j < BUSY_TEST_TRIES; j++)
		{
			if ((ret_val = GetStatus(card_ndx)) < 0)
				continue;

			if ((cmd_status & UPC2_DSP_BUSY) == 0)
				break;
		}
		if (j == BUSY_TEST_TRIES)
		{
			ret_val = UPC2_COMM_ERR;
			continue;
		}

		// Break out if bad CRC
		if (cmd_status & UPC2_DSP_BAD_CRC)
		{
			ret_val = UPC2_COMM_ERR;
			break;
		}

		// Break out if completed OK
		if (cmd_status & UPC2_DSP_COMPLETED_OK)
		{
			ret_val = UPC2_NORMAL_RETURN;
			break;
		}

		// Set ret_val if completed NG
		if (cmd_status & UPC2_DSP_COMPLETED_NG)
			ret_val = UPC2_DSP_COMMAND_NG;

	}
	if (i == SEND_CMD_MAX_TRIES)
		ret_val = UPC2_EXCEEDED_CMD_RETRY_LIMIT;

#endif
	if (ret_val == UPC2_NORMAL_RETURN)
	{
		// Copy UPC2_SysInfo from command buffer to callers area
		ReadFromLocalAddressSpace(card_ndx, COMMAND_DATA_BUFFER_ADDR,
										  pOPinfo, sizeof(op_info_t));
	}
	return ret_val;

}
/////////////////////////////////////////////////////////////////////////////////////////////////
//
// UPC2_PCI_RunDiagnostics -- sends a command to run the commanded diagnostic.
//
//	parameters
//
//  card_ndx 			-- long 0, 1, 2, .. representing the card's index
//
//  command	-- TBD
//
//  addr 	-- pointer to a buffer TBD 
//
//	size	-- size in bytes of the buffer pointed to by addr.
//
//	status	-- result of running the diagnostic TBD
//
// Returns -- negative if an error occurs.
//			  UPC2_COMM_ERR 		communication failure
//			  UPC2_BUSY 			data collection in progress. 
//			  UPC2_INVALID_INDEX 	if no UPC card with the specified index
//
DllExport long __stdcall UPC2_PCI_RunDiagnostics(long card_ndx, long command, void * addr , long size, long * status)
{
	//return TRUE;
	return 1;
}
