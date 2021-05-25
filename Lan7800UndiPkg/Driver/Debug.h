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

#ifndef _LAN7800_DEBUG_H_
#define _LAN7800_DEBUG_H_

//Debug message levels
#define DBG_WARNING		0x00000001
#define DBG_ERROR		0x00000002
#define DBG_2021		0x00000004
#define DBG_VERSION		0x00000008
#define DBG_TRACE		0x00000010

// Modules enable
#define DBG_DRV			0x10000000
#define DBG_LAN			0x20000000
#define DBG_UNDI		0x40000000

extern int DebugMode;

UINTN
MyDebug(
	CHAR8	*AStr,
	...
	);

#define MYDEBUG

#ifdef MYDEBUG
#define DEBUGPRINT(lvl, msg) \
            if ((DebugMode & lvl) != 0) MyDebug msg
#else
#define DEBUGPRINT
#endif


#endif  /* _LAN7800_DEBUG_H_ */
