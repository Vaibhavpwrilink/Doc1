9-22-05

Version 1.0.0.7 has correction for the case when GetData wraps past the
end of the circular buffer.
=============================================================================    
9-22-05

Version 1.0.0.8 checks the CRC following the data frame
=============================================================================    
11-08-05

Version 1.0.0.9 removed debug code that erroneously set FrameAddr using newFrameAddr 
from last access.
=============================================================================    
11-16-05
Version 1.0.0.10 

(1) Added ReadWithCheckFromLocalAddressSpace that performs verification of the check word and 
    does retries if necessary.

(2) Simplified the handling in GetData of Easysense vs NO_GAPS.

(3) Assumes 32-bit checksum for check word (rather than CRC).
=============================================================================    
10-19-06
Version 1.0.0.11

(1) Added UPC2_PCI_GetUnreadFrameCount to support AxUPCServer for Teledyne

(2) Added upc2_pci.odl in order to create upc2_pci.tlb to avoid VB problem with 
    UPC2_PCI_GetData function when it is Declare'ed.
=============================================================================    
9-17-07
Version 1.0.0.12

Added UPC2_FROM_LOAD_PTR (=5) access type to allow the caller to load frames before the
scan intervall has been counted down.
=============================================================================
1-11-08
Version 1.0.0.13

Added code to Map_BAR to determine if the UPC card has been modified
to support 2282 DMA and sets address offset factor if necessary.
=============================================================================
8-05-08
version 1.0.0.14

Removed 1.0.0.13 modification to Map_BAR
Added code to perform a reset the PCI upon connecting.
=============================================================================
9-10-08
version 1.0.0.15

(1) Removed software handshake
(2) Removed PCI reset on inital connection
(3) Added flush of write buffer (by reading HPIA) at the conclusion of WriteToLocalAddressSpace
(4) Doubled the wait time for burning in UPC2_PCI_InCircuitProgram
=============================================================================
6-19-09
 version 1.0.0.16  

Universal DLL (works with original UPC2100 boards and boards modified for USB2250)

Added a test in Map_BAR to determine if the UPC2100 has been modified for USB2250
use. If mod is present then the HPI offsets are multiplied by 0x200.
=============================================================================
7-16-09
 version 1.0.0.17  

Modified UPC2_PCI_ReadSystemSerialNumber to use the serial_number that is stored in the
DSP flash memory if the version number is 1.7 or later. Otherwise it reads the serial
number from PLX 9030 EEPROM.

Retained UPC2_PCI_ReadSystemSerialNumberFromEEPROM for direct access to EEPROM.
=============================================================================
11-05-09
 version 1.0.0.18

Modified UPC2_PCI_GetSystemInventory to Stop Data Collection before reading the system
serial number. This is required because it is a DSP command. 
=============================================================================
2-03-10
 version 1.0.0.19 for Teledyne Only

Added  float   voltage_offset[8];      // MaxG 2-01-10 offset for chs 1, 2, ...,8
to Calibration data structure in upc2_def.h 

Requires Teledyne version of the DSP code (v 2.0)
=============================================================================
5-10-10
 version 1.0.0.20 for Teledyne Only

Reinstated PCI Reset as part of MapBAR (corrects Teledyne problem)

Requires Teledyne version of the DSP code (v 2.0)
=============================================================================
5-11-10
 version 1.0.0.21

Reinstated PCI Reset as part of MapBAR

Requires nonTeledyne version of the DSP code (v 1.7)
=============================================================================