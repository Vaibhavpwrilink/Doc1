//////////////////////////////////////////////////////////////////////////
//
//	upc2_def.h -- definitions shared by DSP application and interface DLL
//
//
//
///////////////////////////////////////////////////////////////////////////
//                          #Defines                                     // 
///////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C"   {
#endif

// MaxG 2-03-10 Added eight single precision FP voltage offsets (one per channel)
//              to calib_data struct for Teledyne.
//
//              This affects the IRAM map (but not the Flash map)
//
//#define TELEDYNE_SPECIAL      
#define Int32 int

// For testing as Win32 console app
#define Uint32 unsigned int
#define BOOL int

#define MAX_SBITS 			400
#define MAX_ITEMS 			24
#define MAX_AVG_COUNT		5000

#define MAX_USB_CARDS 		10

#define MAX_PCI_CARDS 		10
#define CRC32_POLYNOMIAL    0xEDB88320L
#define SEND_CMD_MAX_TRIES	3
#define READ_DATA_MAX_TRIES	3
#define BUSY_TEST_TRIES		100

// Addresses in IRAM
#define IRAM_BASE 		   					0
#define TIMESTAMP_ADDR						(IRAM_BASE)

#define END_TO_END		1

#ifdef END_TO_END
#define SDRAM_MEMORY_MAP_ADDR			   	(IRAM_BASE + 32)
#define COMMAND_BUFFER_ADDR					(SDRAM_MEMORY_MAP_ADDR + sizeof(UPC2_SDRAM_MemoryMap_t))
#define UPC2_CALIB_STRUCT_TABLE_ADDR        (COMMAND_BUFFER_ADDR + sizeof(UPC2_CommandBuffer_t))
#define UPC2_CONFIG_STRUCT_ADDR				(UPC2_CALIB_STRUCT_TABLE_ADDR + sizeof(UPC2_Calib_data_t))

#define CONVERTED_DATA_FRAMES_POOL_HDR_ADDR	(UPC2_CONFIG_STRUCT_ADDR + sizeof(UPC2_Config_t))
// 11-01-05 MaxG added 4 bytes for 32-bit checkword
#define MOVING_AVERAGE_HDR_ADDR		  		(CONVERTED_DATA_FRAMES_POOL_HDR_ADDR + 4 + MAX_ITEMS * sizeof(UPC2_ConvertedDataFramePoolHdr_t))

#else
#define SDRAM_MEMORY_MAP_ADDR			   	(0x100)
#define COMMAND_BUFFER_ADDR					(SDRAM_MEMORY_MAP_ADDR + sizeof(UPC2_SDRAM_MemoryMap_t))
#define UPC2_CALIB_STRUCT_TABLE_ADDR		(0x200)
#define UPC2_CONFIG_STRUCT_ADDR				(0x300)

#define CONVERTED_DATA_FRAMES_POOL_HDR_ADDR	(0x2000)
#define MOVING_AVERAGE_HDR_ADDR		  		(0x2400)
#endif

#define END_OF_IRAM_USAGE					(MOVING_AVERAGE_HDR_ADDR + MAX_ITEMS * sizeof(UPC2_MovingAverageBufferHdr_t))

// Addresses in flash memory (and parameters)
#define FLASH_SECTOR_SIZE_IN_WORDS			32768

#define BOOT_FLASH_BASE			   			0x90000000	

#define SECONDARY_FLASH_BASE			   	0x90000000	
#define CALIB_CONFIG_AREA1_ADDR				(SECONDARY_FLASH_BASE + 0x4000)
#define CALIB_CONFIG_AREA2_ADDR				(SECONDARY_FLASH_BASE + 0x6000)
#define CALIB_CONFIG_AREA_SIZE				0x1FFC

#define SERIAL_NUMBER_PREAMBLE_ADDR         0x90008000 
#define SERIAL_NUMBER_ADDR                  (SERIAL_NUMBER_PREAMBLE_ADDR + 4)

#define PRIMARY_FLASH_BASE			   		0x90010000	
#define PGM_CODE_AREA1_ADDR					(PRIMARY_FLASH_BASE)
#define PGM_CODE_AREA2_ADDR					(PRIMARY_FLASH_BASE + 0x40000)
#define PGM_CODE_AREA_SIZE					0x3FFFC 

// Offsets in flash memory (in bytes)
#define CALIB_OFFSET						32
#define CONFIG_OFFSET						200

// Addresses in SDRAM
#define SDRAM_BASE 						   	0xa0000000	
#define SDRAM_BUFFER_BASE 	 			   	(SDRAM_BASE + 0x85000)
#define MOVING_AVERAGE_BUFFER_BASE  		(SDRAM_BUFFER_BASE + 15204)

//	space for MAX_ITEMS * MAX_AVG_COUNT * 4 = 24 * 5000 * 4 = 480,000

#define CONVERTED_DATA_FRAMES_ADDR		   	(SDRAM_BUFFER_BASE + 1200028)
#define COMMAND_DATA_BUFFER_ADDR	  	   	(SDRAM_BUFFER_BASE + 1200028)
#define SDRAM_END							0xa2000000


// for DEBUG limit to 4 MB
//#define SDRAM_END							0xa0400000

// Addresses within SDRAM_MEMORY_MAP

//#define COMMAND_BUFFER_MM_ADDR					SDRAM_MEMORY_MAP_ADDR
#define UPC2_CALIB_STRUCT_TABLE_MM_ADDR			(SDRAM_MEMORY_MAP_ADDR+4)
#define UPC2_CONFIG_STRUCT_MM_ADDR				(SDRAM_MEMORY_MAP_ADDR+8)
//#define MOVING_AVERAGE_MAP_MM_ADDR		  		(SDRAM_MEMORY_MAP_ADDR+12)
//#define CONVERTED_DATA_FRAMES_POOL_HDR_MM_ADDR	(SDRAM_MEMORY_MAP_ADDR+16)

// Return values (from DLL to PC) 
#define UPC2_NORMAL_RETURN					1

#define UPC2_COMM_ERR 						-1

#define UPC2_NULL_PARAM	     				-2
#define UPC2_NO_VALID_DEVICE				-3
#define UPC2_NO_ACTIVE_DRIVER				-4
#define UPC2_NO_PLX_IN_NTH_POSITION			-5
#define UPC2_INVALID_INDEX					-6
#define UPC2_NO_CONNECTION					-7

#define UPC2_BUSY 							-10
#define UPC2_NO_CONFIG						-11
#define UPC2_DATA_COLLECTION_NOT_STARTED	-12
#define UPC2_PROGRAM_ERR					-13
#define UPC2_NO_CALIB_DATA					-14
#define UPC2_NO_SYS_INFO					-15

#define UPC2_DSP_COMMAND_NG					-16
#define UPC2_BAD_CRC						-17

#define UPC2_EXCEEDED_CMD_RETRY_LIMIT		-18

#define UPC2_FILE_OPEN_ERR					-19
#define UPC2_PGM_CRC_ERR					-20
#define UPC2_HEX_FILE_ERR					-21
#define UPC2_CKSUM_ERR 		 				-22
#define UPC2_CONFIG_NOT_FOUND 				-23

#define UPC2_ODD_NUMBER_OF_SBITS			-24
#define UPC2_NO_SERIAL_NUMBER_PRESENT		-25
#define UPC2_DSP_VER_LT_1_7	                -26

#define UPC2_UNABLE_TO_MAP_BAR			        -30
#define UPC2_COMMAND_NOT_ALLOWED_IN_DEMO_MODE   -31
#define UPC2_IN_DEMO_MODE                       -32


#define UPC2_INVALID_ITEM			            -40


// DSP Commands
#define UPC2_DSP_START_DATA_COLLECTION				0x10000000
#define UPC2_DSP_STOP_DATA_COLLECTION				0x11000000

#define UPC2_DSP_LOAD_CONFIGURATION_FROM_FLASH		0x20000000
#define UPC2_DSP_SAVE_CONFIGURATION_TO_FLASH		0x21000000

#define UPC2_DSP_IN_CIRCUIT_PROGRAM					0x30000000
#define UPC2_DSP_DOWNLOAD_PROGRAM  					0x31000000

#define UPC2_DSP_UPLOAD_CALIBRATION_DATA			0x40000000
#define UPC2_DSP_DOWNLOAD_CALIBRATION_DATA			0x41000000

#define UPC2_DSP_LOAD_CALIBRATION_DATA_FROM_FLASH	0x50000000
#define UPC2_DSP_SAVE_CALIBRATION_DATA_TO_FLASH		0x51000000

#define UPC2_DSP_GET_SYSTEM_INFO					0x70000000
#define UPC2_DSP_GET_SYSTEM_OP_INFO					0x71000000
#define UPC2_DSP_RUN_DIAGNOSTIC_TEST				0x72000000

#define UPC2_DSP_GET_SERIAL_NUMBER				    0x80000000
#define UPC2_DSP_SET_SERIAL_NUMBER				    0x81000000

// Flash programming destinations

#define	UPC2_DSP_BOOT		0x424F4F54
#define	UPC2_DSP_PGM		0x0050474D


// DSP Command Status (masks)
#define	UPC2_DSP_NEW_COMMAND				0x00000100
#define	UPC2_DSP_INVALID_COMMAND		 	0x00000020
#define	UPC2_DSP_COLLECTING_DATA		 	0x00000010
#define	UPC2_DSP_PROCESSING_COMMAND			0x00000008
#define	UPC2_DSP_BAD_CRC					0x00000004
#define	UPC2_DSP_COMPLETED_OK				0x00000002
#define	UPC2_DSP_COMPLETED_NG				0x00000001

#define UPC2_DSP_BUSY  (UPC2_DSP_NEW_COMMAND | UPC2_DSP_COLLECTING_DATA | UPC2_DSP_PROCESSING_COMMAND)

// Access types
#define	UPC2_NO_GAPS       	    0x00000002
#define	UPC2_FROM_START_FRAME  	0x00000003
#define	UPC2_NEWEST_DATA		0x00000004
#define	UPC2_FROM_LOAD_PTR		0x00000005

// DSP Converted Data Frame Status (masks)
#define	UPC2_DSP_DATA_OVERRUN	0x00000004
#define	UPC2_DSP_DATA_CONSUMED	0x00000002
#define	UPC2_DSP_DATA_LOADED 	0x00000001


// PCI Operational states (masks)
#define	UPC2_CONNECTED			0x00000001

// DSP Operational states (masks)
#define	DSP_DATA_COLLECTION_STARTED		0x00000001


///////////////////////////////////////////////////////////////////////////
//                          Structs                                      // 
///////////////////////////////////////////////////////////////////////////
//
//	N O T E :  All structures must occupy a multiple of 32 bits
//		       to accomodate 32-bit virtual transfers.
//

// Command buffer layout
typedef struct
{
   Uint32 	CRC;
   Int32 	command;
   Int32 	parameter[2];
   Int32 	control_status;
} UPC2_CommandBuffer_t;

// Calib / Config header 
typedef struct
{
   Int32		calib_offset;	// offset (in bytes) (from area base) to UPC2_Calib_data structure
							 	//   = -1 if no calib data  
   Int32		config_offset;	// offset (in bytes) (from area base) to UPC2_Config structure
								//   = -1 if no calib data  
} UPC2_Calib_Config_Hdr_t;

// Moving average buffer header (one per item)
typedef struct
{
   double	sum;
   double	fac;		// = 1 / max_count
   float	average;
   Int32	max_count;
   Int32	count;
   float *	pBuffStart;
   float *	pBuffEnd;
   float *	pCurrEntry;
} UPC2_MovingAverageBufferHdr_t;

// Converted data structures
typedef struct
{
   Int32 	frame_no;
   Int32 	timestamp;			// time in microseconds x 10  
   float  	data[1];
} UPC2_ConvertedDataFrame_t;

#define EZ_SENSE_FRAME_SIZE  (sizeof(UPC2_ConvertedDataFrame_t) + ((MAX_ITEMS-1) * sizeof(float)))

typedef struct
{
   Int32 	frame_no;
   Int32 	timestamp;			// time in microseconds x 10
   short  	data[1];
} UPC2_RawDataFrame_t;


typedef struct
{
   UPC2_ConvertedDataFrame_t  *	pFrame1;	// Pointer to Converted Data Frame 1
   UPC2_ConvertedDataFrame_t  *	pLoad;  	// Load Pointer (points to next frame for data storage)
   UPC2_ConvertedDataFrame_t  *	pNew; 	  	// Consumer Pointer to newest data frame
   UPC2_ConvertedDataFrame_t  *	pStart;	  	// Consumer Pointer to start frame
   UPC2_ConvertedDataFrame_t  *	pLast; 	  	// Pointer to last frame in buffer
   Int32					   	FrameSize;  // including header (in bytes)
   Int32					   	MaxFrames;
} UPC2_ConvertedDataFramePoolHdr_t;

// Offsets within pool header
#define PNEW_OFFSET		8
#define PSTART_OFFSET	12

// Item definition
typedef struct
{
   Int32 item_number;		    	   // base 0
   Int32 terminal_block_number;		   // base 0 used by RTD proc to pick up 5 K Ohm resistance value
   Int32 gui_index1;                   // xx ss tt gg
   Int32 gui_index2;                   // xx pp c1 c2  
									   //  pp = precision drop down index, c1 = , c2 = 
   float scale_factor;
   float offset;
   Int32 avg_count;					   // zero if not averaging MAX = 5000
   char  tag[64];					   // to allow for BSTR header
   char  engineering_units[8];
   Int32 reserved[3];                  // (put filter divisor here)
   float value;					       // (scaled and averaged if specified)	
   Int32 recipe_code;				   // ssttxx = indexes in the drop-down lists
									   //    ss=Sensor, tt=Type, xx = don't care
   Int32 related_item_flag;	  		   // NZ if this is a related item
   Int32 related_item_number;	  	   // base 0  (-1 if NO related item)
   float related_item_value;		   // storage for converted related item 			 		
   Int32 number_of_conversions_used;   // 1 to 4 (not counting the related item)
   Int32 index_of_conversion_used[4];  // base 0 
   Int32 index_of_gain_used[4];		   // base 0 (0 = 20 mv, ... , 9 = 10.24 v)
} item_t;

// Config structure
typedef struct
{
    char   description[64];	  		    // null terminated
    char   creation_date[12]; 		    // mmm dd yyyy\0
    Int32  nItems;
    Int32  nSbits;
    Int32  sine_wave_frequency;		    // 0, 2500 or 5000
    Int32  vOut;						// 5 or 10 v
	Int32  filter_divisor;				// 0 if not used (used in calibration)
    Int32  McBSP0_clk_div;	         	// 40 => 100 KHz, 200 => 20 KHz
    Int32  scans_remaining;		
    Int32  scan_interval;			// multiple of frame period
    Int32  starting_row;
    Int32  sel_boxes;	   // xx xx ts rd (8 hex digits)
						   // t = index in drop down box for time units
						   //      0 = usecs, 1 = msecs, 2 = secs
						   // s if = 1 => scans remaining checkbox checked
						   // r if = 1 => row increment checkbox checked
						   // d 1 = Excel, 2 = ASCII

	Int32  op_flags;		// xx xx xx cc (8 hex digits)
						    // cc = 52 (hex) ( = 'R') if returns raw data 
					   	    // cc = 49 (hex) ( = 'I') if returns integer data 
	Int32  reserved[2];
    Int32  sbits[MAX_SBITS];
    item_t item[MAX_ITEMS];
} UPC2_Config_t;

// Raw data structure
typedef struct
{
  Int32	timestamp;
  Int32	data[MAX_SBITS];
} raw_data_t;

// Software info structure
typedef struct
{
    char version[32];				// null terminated
    char creation_date[12]; 		// mmm dd yyyy\0
    char creation_time[12];			// hh:mm:ss\0
    Int32 code_size;
} sw_info_t;

// System Op info structure
typedef struct
{
    Int32 pSbitTbl;	              // local address of SbitTbl
    Int32 pInit_oldch_data;		  // local address of init_oldch_data
    Int32 pOldch_data;		      // local address of oldch.data
    Int32 pInt_count;			  // local address of int_count
    Int32 pFilter_divisor;		  // local address of filter_divisor
} op_info_t;


// Calibration data structure
typedef struct
{
    float   internal_5K_res_value[8];
    float   conversion_factor[10];
    Int32   conversion_offset[10];
#ifdef TELEDYNE_SPECIAL
    float   voltage_offset[8];      // MaxG 2-01-10 offset for chs 1, 2, ...,8
#endif
    char    calibration_date[12];   // mm dd yyyy\0
} UPC2_Calib_data_t;

// SDRAM memory map header
typedef struct
{	
   UPC2_CommandBuffer_t 			*	pCommandBuffer;
   UPC2_Calib_data_t 				*	pUPC2_Calib;
   UPC2_Config_t 					*	pUPC2_Config;
   UPC2_ConvertedDataFramePoolHdr_t * 	pConvertedDataFrameHdr;
   UPC2_MovingAverageBufferHdr_t	*	pMovingAverageBufferHdr;
   UPC2_ConvertedDataFrame_t 		* 	pConvertedDataFrames;		
   Int32  								reserved[3];
} UPC2_SDRAM_MemoryMap_t;


#ifdef __cplusplus
}
#endif
//////////////////////// End Of File ////////////////////////

