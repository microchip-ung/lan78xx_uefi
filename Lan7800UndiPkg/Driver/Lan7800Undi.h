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

#ifndef _LAN7800_UNDI_H_
#define _LAN7800_UNDI_H_


VOID
Lan7800UndiPxeGetState(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeStart(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
  );

VOID
Lan7800UndiPxeStop(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
  );

VOID
Lan7800UndiPxeGetInitInfo(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeGetConfigInfo(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeInitialize(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeReset(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeShutdown(
	LAN7800_ADAPTER_DATA    *Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeInterruptEnables(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeReceiveFilters(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeStationAddress(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeStatistics(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeMcastIpToMac(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeNvData(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeGetStatus(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeFillHeader(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeTransmit(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

VOID
Lan7800UndiPxeReceive(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

EFI_STATUS
Lan7800UndiModuleInit(
	EFI_LOADED_IMAGE_PROTOCOL	*LoadedImage
	);

EFI_STATUS
Lan7800UndiProtocolInitialize(
	VOID	*pAdapterInstance
  );

EFI_STATUS 
Lan7800UndiDeviceDeInitialize(
	LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS
Lan7800UndiPxeReceiveData(
	LAN7800_ADAPTER_DATA	*Adapter,
	UINT8					*Buffer,
	PXE_UINT32				BufferLen,
	PXE_UINT32				*FrameLen
	);

#endif /* _LAN7800_UNDI_H_ */
