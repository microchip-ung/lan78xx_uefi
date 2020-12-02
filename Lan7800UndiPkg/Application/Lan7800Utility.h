/*

    Copyright (C) 2009 - 2020. Microchip Technology Inc. and its 
            subsidiaries (Microchip).  All rights reserved.

    You are permitted to use the software and its derivatives with Microchip 
    products. See the license agreement accompanying this software, if any, 
    for additional info regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY 
    KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
    WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
    PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS 
    BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
    CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR 
    ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
    ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER 
    SIMILAR COSTS. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP AND ITS 
    LICENSORS LIABILITY WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU 
    PAID DIRECTLY TO MICROCHIP TO USE THIS SOFTWARE. MICROCHIP PROVIDES THIS 
    SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.  
*/

#ifndef _LAN7800_UTILITY_H_
#define _LAN7800_UTILITY_H_

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/BaseMemoryLib.h>
#include <Protocol/Lan7800CmdProtocol.h>

#define TEMP_STRING_LENGTH		0x80

#define TITLE_ROW				0
#define TITLE_COLUMN			0
#define CURRENT_DEVICE_ROW		2
#define CURRENT_DEVICE_COLUMN	0

#define EEPROM_ONE_PAGE_MAX_LEN	256
#define EEPROM_ONE_ROW_MAX_LEN	16

#define EEPROM_AREA_BASE_ROW	4
#define EEPROM_AREA_BASE_COL	2
#define EEPROM_AREA_DATA_ROW	5
#define EEPROM_AREA_DATA_COL	7
#define EEPROM_AREA_CHAR_ROW	5
#define EEPROM_AREA_CHAR_COL	58
#define EEPROM_AREA_HELP_COL	0
#define EEPROM_AREA_HELP_ROW	23

typedef struct {
	EFI_STATUS (*ConfigFunction)(LAN7800_DEVICE_CMD_PROTOCOL*, CHAR16*);
	CHAR16  *Description;
} CONFIG_FUNCTION;


EFI_STATUS
ViewEditEeprom(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	);

EFI_STATUS
ViewProperty(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	);

EFI_STATUS
ViewEditPhyRegister(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
  );

EFI_STATUS
ViewEditRegister(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	);

EFI_STATUS
DumpOtp(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	);

#endif /* _LAN7800_UTILITY_H_ */

