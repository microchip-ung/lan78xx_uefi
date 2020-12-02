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

#include "Lan7800Driver.h"
#include "Lan7800Undi.h"
#include "Lan7800.h"
#include "Debug.h"

//NII version
#define LAN7800_NII_VERSION		1

//Device Undi states
#define UNDI_STATE_STOPPED		(1 << 0)
#define UNDI_STATE_STARTED		(1 << 1)
#define UNDI_STATE_INITIALIZED	(1 << 2)
#define UNDI_STATE_ANY			(UNDI_STATE_STOPPED | UNDI_STATE_STARTED | UNDI_STATE_INITIALIZED)

#define PXE_OPFLAGS_NOT_USED_CHECK_MASK				(PXE_UINT16)(~PXE_OPFLAGS_NOT_USED)
#define PXE_OPFLAGS_INITIALIZE_OPFLAGS_CHECK_MASK	(PXE_UINT16)(~PXE_OPFLAGS_INITIALIZE_CABLE_DETECT_MASK)
#define PXE_OPFLAGS_RESET_CHECK_MASK				(PXE_UINT16)(~(PXE_OPFLAGS_RESET_DISABLE_FILTERS | PXE_OPFLAGS_RESET_DISABLE_INTERRUPTS))
#define PXE_OPFLAGS_INTERRUPT_CHECK_MASK			(PXE_UINT16)(~PXE_OPFLAGS_INTERRUPT_OPMASK)
#define PXE_OPFLAGS_RECEIVE_FILTER_CHECK_MASK		0x1FE0
#define PXE_OPFLAGS_STATION_ADDRESS_CHECK_MASK		(PXE_UINT16)(~PXE_OPFLAGS_STATION_ADDRESS_RESET)
#define PXE_OPFLAGS_STATISTICS_CHECK_MASK			(PXE_UINT16)(~PXE_OPFLAGS_STATISTICS_RESET)
#define PXE_OPFLAGS_GET_STATUS_CHECK_MASK			(PXE_UINT16)(~(PXE_OPFLAGS_GET_INTERRUPT_STATUS | PXE_OPFLAGS_GET_TRANSMITTED_BUFFERS))
#define PXE_OPFLAGS_FILL_HEADER_CHECK_MASK			(PXE_UINT16)(~PXE_OPFLAGS_FILL_HEADER_OPMASK)
#define PXE_OPFLAGS_TRANSMIT_CHECK_MASK				(PXE_UINT16)(~(PXE_OPFLAGS_TRANSMIT_OPMASK | PXE_OPFLAGS_SWUNDI_TRANSMIT_OPMASK))
#define SIZE_NOT_CHECK								(0xFFFF)

//device instances
LAN7800_ADAPTER_DATA *gLan7800UndiDevices[LAN7800_MAX_DEVICES] = {0};
//Number of devices in the Lan7800UndiDevices
UINT8 gLan7800UndiDeviceCount = 0;

UINT64 gLan7800ImageAddr = 0;
UINT32 gLan7800ImageSize = 0;

//PXE SW UNDI 
PXE_SW_UNDI	*gLan7800PxeSwUndiBuffer = NULL;
PXE_SW_UNDI	*gLan7800PxeSwUndi = NULL;


typedef void (*UNDI_API)(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	);

typedef struct _UNDI_CALL_TABLE {
	UINT8			RequiredUndiState;
	PXE_OPFLAGS		OpFlagsMask;
	PXE_UINT16		CPBsize;
	PXE_UINT16		DBsize;
	UNDI_API		FunctionPtr;
} UNDI_CALL_TABLE;

static UNDI_CALL_TABLE UndiCallTable[] = {
	{ // 0x00 UndiPxeGetState
		UNDI_STATE_ANY,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeGetState
	},
  
	{ // 0x01 UndiPxeStart
		UNDI_STATE_STOPPED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		sizeof(PXE_CPB_START_31),
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeStart
	},
  
	{ // 0x02 UndiPxeStop
		UNDI_STATE_STARTED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeStop
	},
  
	{ // 0x03 UndiPxeGetInitInfo
		UNDI_STATE_STARTED | UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		sizeof(PXE_DB_GET_INIT_INFO),
		Lan7800UndiPxeGetInitInfo
  },
  
  { // 0x04 UndiPxeGetConfigInfo
		UNDI_STATE_STARTED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		sizeof(PXE_DB_GET_CONFIG_INFO),
		Lan7800UndiPxeGetConfigInfo
	},
    
	{ // 0x05 UndiPxeInitialize
		UNDI_STATE_STARTED,
		PXE_OPFLAGS_INITIALIZE_OPFLAGS_CHECK_MASK,
		sizeof(PXE_CPB_INITIALIZE),
		sizeof(PXE_DB_INITIALIZE),
		Lan7800UndiPxeInitialize
	},
  
	{ // 0x06 UndiPxeReset
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_RESET_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeReset
	  },
    
	{ // 0x07 UndiPxeShutdown
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeShutdown
	},
  
	{ // 0x08 UndiPxeInterruptEnables
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_INTERRUPT_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeInterruptEnables
	},
    
	{ // 0x09 UndiPxeReceiveFilters
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_RECEIVE_FILTER_CHECK_MASK,
		SIZE_NOT_CHECK,
		SIZE_NOT_CHECK,
		Lan7800UndiPxeReceiveFilters
	},
  
	{ // 0x0A UndiPxeStationAddress
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_STATION_ADDRESS_CHECK_MASK,
		SIZE_NOT_CHECK,
		SIZE_NOT_CHECK,
		Lan7800UndiPxeStationAddress
	},
    
	{ //0x0B UndiPxeStatistics
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_STATISTICS_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		SIZE_NOT_CHECK,
		Lan7800UndiPxeStatistics
	},
  
	{ //0x0C UndiPxeMcastIpToMac
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_MCAST_IP_TO_MAC_OPMASK,
		sizeof(PXE_CPB_MCAST_IP_TO_MAC),
		sizeof(PXE_DB_MCAST_IP_TO_MAC),
		Lan7800UndiPxeMcastIpToMac
	},
  
	{ //0x0D UndiPxeNvData
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_NVDATA_OPMASK,
		SIZE_NOT_CHECK,
		sizeof(PXE_DB_NVDATA),
		Lan7800UndiPxeNvData
	},
  
	{ //0x0E UndiPxeGetStatus
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_GET_STATUS_CHECK_MASK,
		PXE_CPBSIZE_NOT_USED,
		SIZE_NOT_CHECK,
		Lan7800UndiPxeGetStatus
	},
  
	{ //0x0F UndiPxeFillHeader
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_FILL_HEADER_CHECK_MASK,
		SIZE_NOT_CHECK,
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeFillHeader
	},
  
	{ //0x10 UndiPxeTransmit
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_TRANSMIT_CHECK_MASK,
		sizeof(PXE_CPB_TRANSMIT),
		PXE_DBSIZE_NOT_USED,
		Lan7800UndiPxeTransmit
	},
  
	{ //0x11 UndiPxeReceive
		UNDI_STATE_INITIALIZED,
		PXE_OPFLAGS_NOT_USED_CHECK_MASK,
		sizeof(PXE_CPB_RECEIVE),
		sizeof(PXE_DB_RECEIVE),
		Lan7800UndiPxeReceive
	},
};

VOID
UpdateInterfaceCount(
	VOID
	)
{
	UINT8	Index;
	UINT8	MaxIfNum = 0xff;

	for (Index = 0; Index < LAN7800_MAX_DEVICES; Index++) {
		if (gLan7800UndiDevices[Index] != NULL)
			MaxIfNum = Index;  
	}

	gLan7800UndiDeviceCount = (MaxIfNum + 1);
}

UINT8
GetInterfaceCount(
	VOID
	)
{
	return gLan7800UndiDeviceCount;
}

UINT8
GetInterfaceNumber(
	LAN7800_ADAPTER_DATA    *Adapter
  )
{
	UINT8	Index;
  
	for (Index = 0; Index < LAN7800_MAX_DEVICES; Index++) {
		if(gLan7800UndiDevices[Index] == NULL) {
			gLan7800UndiDevices[Index] = Adapter;
			UpdateInterfaceCount();
			return Index;
		}
	}

	return 0xFF;
}

EFI_STATUS
RemoveInterface(
	LAN7800_ADAPTER_DATA    *Adapter
	)
{
	UINT8	Index;
  
	for (Index = 0; Index < LAN7800_MAX_DEVICES; Index++) {
		if(gLan7800UndiDevices[Index] == Adapter) {
			gLan7800UndiDevices[Index] = NULL;
			UpdateInterfaceCount();
			return EFI_SUCCESS;
		}
	}
	return EFI_NOT_FOUND;
}

VOID
Lan7800UndiPxeGetState(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_STATFLAGS	StatFlags;
  
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	switch (Adapter->UndiState) {
		case UNDI_STATE_STOPPED :
			StatFlags = PXE_STATFLAGS_GET_STATE_STOPPED;
			break;

		case UNDI_STATE_STARTED :
			StatFlags = PXE_STATFLAGS_GET_STATE_STARTED;
			break;

		default :
			StatFlags = PXE_STATFLAGS_GET_STATE_INITIALIZED;
			break;
	}
  
	Cdb->StatFlags = StatFlags | PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;  
}

VOID
Lan7800UndiPxeStart(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_CPB_START_31	*CpbStart_31;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	CpbStart_31 = (PXE_CPB_START_31 *)(UINTN)Cdb->CPBaddr;
  
	Adapter->Delay		= (PXE31_DELAY)(UINTN)CpbStart_31->Delay;
	Adapter->Block		= (PXE31_BLOCK)(UINTN)CpbStart_31->Block;
	Adapter->Vir2Phys	= (PXE31_VIRT2PHYS)(UINTN)CpbStart_31->Virt2Phys;
	Adapter->MemIo		= (PXE31_MEM_IO)(UINTN)CpbStart_31->Mem_IO;
	Adapter->MapMem		= (PXE31_MAP_MEM)(UINTN)CpbStart_31->Map_Mem;
	Adapter->UnMapMem	= (PXE31_UNMAP_MEM)(UINTN)CpbStart_31->UnMap_Mem;
	Adapter->SyncMem	= (PXE31_SYNC_MEM)(UINTN)CpbStart_31->Sync_Mem; 
	Adapter->UniqueId	= CpbStart_31->Unique_ID;

	//Change the state to started
	Adapter->UndiState = UNDI_STATE_STARTED;

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;

	return;
}

VOID
Lan7800UndiPxeStop(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	Adapter->Delay		= NULL;
	Adapter->Block		= NULL;
	Adapter->Vir2Phys	= NULL;
	Adapter->MemIo		= NULL;
	Adapter->MapMem		= NULL;
	Adapter->UnMapMem	= NULL;
	Adapter->SyncMem	= NULL;
	Adapter->UniqueId	= 0;  
	
	//change the state to Stopped
	Adapter->UndiState = UNDI_STATE_STOPPED;
  
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
}

VOID
Lan7800UndiPxeGetInitInfo(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
  )
{
	PXE_DB_GET_INIT_INFO  *DbPtr;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));
  
	DbPtr = (PXE_DB_GET_INIT_INFO *) (UINTN) (Cdb->DBaddr);

	DbPtr->MemoryRequired	= 0;
	DbPtr->FrameDataLen		= PXE_MAX_TXRX_UNIT_ETHER;

    DbPtr->LinkSpeeds[0]	= 10;
    DbPtr->LinkSpeeds[1]	= 100;
    DbPtr->LinkSpeeds[2]	= 1000;
    DbPtr->LinkSpeeds[3]	= 0;

	DbPtr->NvCount			= LAN7800_MAX_EEPROM_LEN;
	DbPtr->NvWidth			= 1;
	DbPtr->MediaHeaderLen	= PXE_MAC_HEADER_LEN_ETHER;
	DbPtr->HWaddrLen		= PXE_HWADDR_LEN_ETHER;
	DbPtr->MCastFilterCnt	= MAX_MCAST_ADDRESS_CNT;

	DbPtr->TxBufCnt			= 1;
	DbPtr->TxBufSize		= Adapter->TxBufferSize;
	DbPtr->RxBufCnt			= 1;
	DbPtr->RxBufSize		= Adapter->RxBufferSize;

	DbPtr->IFtype			= PXE_IFTYPE_ETHERNET;
	DbPtr->SupportedDuplexModes		= PXE_DUPLEX_ENABLE_FULL_SUPPORTED | PXE_DUPLEX_FORCE_FULL_SUPPORTED;
	DbPtr->SupportedLoopBackModes	= PXE_LOOPBACK_INTERNAL_SUPPORTED;

	Cdb->StatFlags |= (PXE_STATFLAGS_CABLE_DETECT_SUPPORTED | PXE_STATFLAGS_GET_STATUS_NO_MEDIA_SUPPORTED);
	Cdb->StatFlags |= PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;

	return ;
}

VOID
Lan7800UndiPxeGetConfigInfo(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_DB_GET_CONFIG_INFO	*DbPtr;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	DbPtr = (PXE_DB_GET_CONFIG_INFO *)(UINTN)Cdb->DBaddr;

	CopyMem(DbPtr, &Adapter->PciConfig, sizeof(PXE_PCI_CONFIG_INFO));

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
}

VOID
Lan7800UndiPxeInitialize(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_CPB_INITIALIZE	*CpbPtr;
	PXE_DB_INITIALIZE	*DbPtr;
	UINT16				LinkDetected = 0;
	PXE_STATFLAGS		StatFlags = 0;
	EFI_STATUS			Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	CpbPtr = (PXE_CPB_INITIALIZE *) (UINTN)Cdb->CPBaddr;
	DbPtr = (PXE_DB_INITIALIZE *) (UINTN)Cdb->DBaddr;

	//if already initialize
	if (Adapter->UndiState == UNDI_STATE_INITIALIZED) {
		DEBUGPRINT(DBG_ERROR, ("Lan7800UndiPxeInitialize: Already initialized\n"));
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_INVALID_CDB;
		return;
	}
  
	DbPtr->MemoryUsed = 0;
	DbPtr->TxBufCnt = 1;
	DbPtr->TxBufSize = Adapter->TxBufferSize;
	DbPtr->RxBufCnt = 1;
	DbPtr->RxBufSize = Adapter->RxBufferSize;

	//Initialize the Rx buffer data preset and count
	Adapter->RxBufferDataPresent = 0;
	Adapter->RxBufferDataLen = 0;
	Adapter->RxBufferDataStart = Adapter->RxBuffer;

	if (Cdb->OpFlags & PXE_OPFLAGS_INITIALIZE_DETECT_CABLE) {
		//do link check
		Status = Lan7800LinkCheck(Adapter, &LinkDetected);
		if (EFI_ERROR(Status)) {
			Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
			Cdb->StatCode = PXE_STATCODE_DEVICE_FAILURE;
			return;
		}
		if (LinkDetected == 0) {
			StatFlags = StatFlags | PXE_STATFLAGS_INITIALIZED_NO_MEDIA;
			Print(L"Lan7800UndiPxeInitialize: Link down\n");
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeInitialize: No link\n", __FUNCTION__));
		} else {
			Print(L"Lan7800UndiPxeInitialize: Link Up\n");
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeInitialize: Link Up\n", __FUNCTION__));
		}
	}
	Adapter->UndiState = UNDI_STATE_INITIALIZED;
  
	Cdb->StatFlags = StatFlags | PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
}

VOID
Lan7800UndiPxeReset(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	Status = Lan7800Reset(Adapter);
	if (Status == EFI_SUCCESS) {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
		Cdb->StatCode = PXE_STATCODE_SUCCESS;
	} else {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_DEVICE_FAILURE;
	}

	return;
}

VOID
Lan7800UndiPxeShutdown(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	Adapter->UndiState = UNDI_STATE_STARTED;

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
}

VOID
Lan7800UndiPxeInterruptEnables(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
}

VOID
Lan7800UndiPxeReceiveFilters(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_DB_RECEIVE_FILTERS	*DbBuf;
	UINTN					MulticastLength = 0;
	
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	//Check if READ
	if (Cdb->OpFlags == PXE_OPFLAGS_RECEIVE_FILTER_READ) {
		if (Cdb->CPBsize != PXE_CPBSIZE_NOT_USED || Cdb->CPBaddr != PXE_CPBADDR_NOT_USED ||
			Cdb->DBsize == PXE_DBSIZE_NOT_USED || Cdb->DBaddr == PXE_DBADDR_NOT_USED) {
			goto BadCdb;
		}

		//Copy adapter Mac addresses to cdb
		DbBuf = (PXE_DB_RECEIVE_FILTERS*) (UINTN)Cdb->DBaddr;
		if (Adapter->MulticastListCount) {
			MulticastLength = Adapter->MulticastListCount * PXE_MAC_LENGTH;
			if (MulticastLength > Cdb->DBsize) {
				MulticastLength = Cdb->DBsize;
			}
			CopyMem(DbBuf, &Adapter->MultiCastList[0], MulticastLength);
		}
		Cdb->StatFlags = Adapter->ReceiveFilterFlags & 0x1F;
		goto CdbDone;
	}

	//if Reset multicast list
	if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST) {
		//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST\n"));
		if (Adapter->MulticastListCount) {
			ZeroMem(&Adapter->MultiCastList[0], (Adapter->MulticastListCount * PXE_MAC_LENGTH));
			Adapter->MulticastListCount = 0;
			Adapter->ReceiveFilterFlags |= PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST;
		}
	}

	//Save multicast filter addresses
	if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) {
		//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST\n"));
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ENABLE) {
			MulticastLength = Cdb->CPBsize;
			if (MulticastLength > (PXE_MAC_LENGTH * MAX_MCAST_ADDRESS_CNT)) {
				MulticastLength = (PXE_MAC_LENGTH * MAX_MCAST_ADDRESS_CNT);
			} 
			CopyMem(Adapter->MultiCastList, (UINT8*)(UINTN)Cdb->CPBaddr, MulticastLength);
			Adapter->MulticastListCount = (UINT8) (MulticastLength / PXE_MAC_LENGTH);
		}
	}

	if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ENABLE) {
		//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_ENABLE\n"));
		//If receive filter needs to be enabled
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_UNICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_UNICAST\n"));
			Adapter->ReceiveFilterFlags |= PXE_STATFLAGS_RECEIVE_FILTER_UNICAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST\n"));
			Adapter->ReceiveFilterFlags |= PXE_STATFLAGS_RECEIVE_FILTER_BROADCAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS\n"));
			Adapter->ReceiveFilterFlags |= PXE_STATFLAGS_RECEIVE_FILTER_PROMISCUOUS;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST\n"));
			Adapter->ReceiveFilterFlags |= PXE_STATFLAGS_RECEIVE_FILTER_ALL_MULTICAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST\n"));
			Adapter->ReceiveFilterFlags |= PXE_STATFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST;
		}
		Adapter->ReceiveFilterFlags |= PXE_OPFLAGS_RECEIVE_FILTER_ENABLE;
	} else if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_DISABLE) {
		//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_DISABLE\n"));
		//If receive filter needs to be disabled
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_UNICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_UNICAST\n"));
			Adapter->ReceiveFilterFlags &= ~PXE_STATFLAGS_RECEIVE_FILTER_UNICAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST\n"));
			Adapter->ReceiveFilterFlags &= ~PXE_STATFLAGS_RECEIVE_FILTER_BROADCAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS\n"));
			Adapter->ReceiveFilterFlags &= ~PXE_STATFLAGS_RECEIVE_FILTER_PROMISCUOUS;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST\n"));
			Adapter->ReceiveFilterFlags &= ~PXE_STATFLAGS_RECEIVE_FILTER_ALL_MULTICAST;
		}
		if (Cdb->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) {
			//DEBUGPRINT(DBG_UNDI, ("UndiPxeReceiveFilters: PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST\n"));
			Adapter->ReceiveFilterFlags &= ~PXE_STATFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST;
		}
		Adapter->ReceiveFilterFlags |= PXE_OPFLAGS_RECEIVE_FILTER_DISABLE;
	}

	Lan7800SetMulticast(Adapter);

CdbDone:
	Cdb->StatFlags |= PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
	return;

BadCdb:
	DEBUGPRINT(DBG_ERROR, ("%a: ERROR!!!\n", __FUNCTION__));
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
	Cdb->StatCode = PXE_STATCODE_INVALID_CDB;
	return;
}


VOID
Lan7800UndiPxeStationAddress(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_CPB_STATION_ADDRESS   *CpbPtr;
	PXE_DB_STATION_ADDRESS    *DbPtr;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if(Cdb->CPBsize != PXE_CPBSIZE_NOT_USED && Cdb->CPBsize != sizeof(PXE_CPB_STATION_ADDRESS)) {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_INVALID_CPB;
		return;
	}
	if (Cdb->DBsize != PXE_DBSIZE_NOT_USED && Cdb->DBsize != sizeof(PXE_DB_STATION_ADDRESS)) {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_INVALID_CPB;
		return;
	}

	//Reset Mac address to permanent address
	if (Cdb->OpFlags & PXE_OPFLAGS_STATION_ADDRESS_RESET) {
		CopyMem(Adapter->MacAddress, Adapter->PermanentMacAddress, PXE_HWADDR_LEN_ETHER);
		Lan7800SetMacAddress(Adapter);
	}

	//Set Mac address to the one got (current)
	if( (Cdb->CPBaddr != PXE_CPBADDR_NOT_USED) && (Cdb->CPBsize != PXE_CPBSIZE_NOT_USED)) {
		CpbPtr = (PXE_CPB_STATION_ADDRESS *) (UINTN) (Cdb->CPBaddr); 
		CopyMem(Adapter->MacAddress, CpbPtr->StationAddr, PXE_HWADDR_LEN_ETHER);
		Lan7800SetMacAddress(Adapter);
	}

	//If DBaddr & DBsize are valid then copy the new addresses 
	if ((Cdb->DBaddr != PXE_DBADDR_NOT_USED) && (Cdb->DBsize != PXE_DBSIZE_NOT_USED)) {
		DbPtr = (PXE_DB_STATION_ADDRESS *) (UINTN) (Cdb->DBaddr);	
		ZeroMem(DbPtr->StationAddr, PXE_MAC_LENGTH);
		ZeroMem(DbPtr->PermanentAddr, PXE_MAC_LENGTH);
		ZeroMem(DbPtr->BroadcastAddr, PXE_MAC_LENGTH);
		CopyMem(DbPtr->StationAddr, Adapter->MacAddress, PXE_MAC_LENGTH);
		CopyMem(DbPtr->PermanentAddr, Adapter->PermanentMacAddress, PXE_MAC_LENGTH);
		CopyMem(DbPtr->BroadcastAddr, Adapter->BroadcastMacAddress, PXE_MAC_LENGTH);
		DEBUGPRINT(DBG_UNDI, ("StationAddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
			DbPtr->StationAddr[0], DbPtr->StationAddr[1], DbPtr->StationAddr[2], 
			DbPtr->StationAddr[3], DbPtr->StationAddr[4], DbPtr->StationAddr[5]));
		DEBUGPRINT(DBG_UNDI, ("PermanentAddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
			DbPtr->PermanentAddr[0], DbPtr->PermanentAddr[1], DbPtr->PermanentAddr[2], 
			DbPtr->PermanentAddr[3], DbPtr->PermanentAddr[4], DbPtr->PermanentAddr[5]));
		DEBUGPRINT(DBG_UNDI, ("BroadcastAddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
			DbPtr->BroadcastAddr[0], DbPtr->BroadcastAddr[1], DbPtr->BroadcastAddr[2], 
			DbPtr->BroadcastAddr[3], DbPtr->BroadcastAddr[4], DbPtr->BroadcastAddr[5]));
	}

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
	return;
}

VOID
Lan7800UndiPxeStatistics(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	Cdb->StatCode = PXE_STATCODE_SUCCESS;
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	return;
}

VOID
Lan7800UndiPxeMcastIpToMac(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{ 
	PXE_CPB_MCAST_IP_TO_MAC *CpbIpPtr;
	PXE_DB_MCAST_IP_TO_MAC  *DbMacPtr;
	UINT8                   *TempPtr;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if (Cdb->CPBsize == PXE_CPBSIZE_NOT_USED || Cdb->CPBaddr == PXE_CPBADDR_NOT_USED ||
			Cdb->DBsize == PXE_DBSIZE_NOT_USED || Cdb->DBaddr == PXE_DBADDR_NOT_USED) {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_INVALID_CDB;
		return;
	}

	CpbIpPtr = (PXE_CPB_MCAST_IP_TO_MAC*)(UINTN)Cdb->CPBaddr;
	DbMacPtr = (PXE_DB_MCAST_IP_TO_MAC*)(UINTN)Cdb->DBaddr;
	TempPtr = (UINT8*)&DbMacPtr->MAC;

	if ((Cdb->OpCode & PXE_OPFLAGS_MCAST_IPV4_TO_MAC) == PXE_OPFLAGS_MCAST_IPV4_TO_MAC) {
		// Convert IPv4 
		TempPtr[0] = 0x01;
		TempPtr[1] = 0x00;
		TempPtr[2] = 0x5E;
		TempPtr[3] = (UINT8)((CpbIpPtr->IP.IPv4 >> 16) & 0x7F);
		TempPtr[4] = (UINT8)((CpbIpPtr->IP.IPv4 >> 8) & 0xFF);
		TempPtr[5] = (UINT8)((CpbIpPtr->IP.IPv4) & 0xFF);   
	} else if ((Cdb->OpCode & PXE_OPFLAGS_MCAST_IPV6_TO_MAC) == PXE_OPFLAGS_MCAST_IPV6_TO_MAC) {
		// Convert IPv6
		TempPtr[0] = 0x33;
		TempPtr[1] = 0x33;
		TempPtr[2] = (UINT8)((CpbIpPtr->IP.IPv6[3] >> 24) & 0xFF);
		TempPtr[3] = (UINT8)((CpbIpPtr->IP.IPv6[3] >> 16) & 0xFF);
		TempPtr[4] = (UINT8)((CpbIpPtr->IP.IPv6[3] >> 8) & 0xFF);
		TempPtr[5] = (UINT8)(CpbIpPtr->IP.IPv6[3] & 0xFF);
	} else {
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode = PXE_STATCODE_UNSUPPORTED;
		return;
	}

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;

	return;
}

VOID
Lan7800UndiPxeNvData(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;

	return;
}

VOID
Lan7800UndiPxeGetStatus(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_DB_GET_STATUS	*DbPtr;
	PXE_STATFLAGS		StatFlags = 0;
//	UINT16				LinkDetected;
	EFI_STATUS			Status = EFI_SUCCESS;
	UINT16				Size;
	UINT32				RxCmdA;
  
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if ((Cdb->DBsize < 16) || (Cdb->DBsize % 8)) {
		DEBUGPRINT(DBG_ERROR, ("%a: Invalid CDB\n", __FUNCTION__));
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		Cdb->StatCode  = PXE_STATCODE_INVALID_CDB;
		return;
	}

	DbPtr = (PXE_DB_GET_STATUS *)(UINTN)Cdb->DBaddr;

	//Try to receive a packet
	if (Adapter->RxTxPathEnabled && (Adapter->RxBufferDataPresent == 0)) {
		Status = Lan7800Receive(Adapter);
	}

	//Check the link status
	
	// 11/24/2020 do not check link status here to improve performance
	/*
	if (Cdb->OpFlags & PXE_OPFLAGS_GET_MEDIA_STATUS) {
		Status = Lan7800LinkCheck(Adapter, &LinkDetected);
		if (EFI_ERROR(Status)) {
			goto UndiPxeGetStatusError;
		}
		if (LinkDetected == 0) {
			StatFlags |= PXE_STATFLAGS_INITIALIZED_NO_MEDIA;
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeGetStatus: No link\n", __FUNCTION__));
		} else {
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeGetStatus: Link Up\n", __FUNCTION__));
		}
	}
	*/
	
	//Indicate the buffer that completed transfer
	if (Cdb->OpFlags & PXE_OPFLAGS_GET_TRANSMITTED_BUFFERS) {
		if (Adapter->TxUndiBuffer != NULL) {
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeGetStatus: PXE_OPFLAGS_GET_TRANSMITTED_BUFFERS\n"));
			DbPtr->TxBuffer[0] = (PXE_UINT64)(UINTN) Adapter->TxUndiBuffer;
			Adapter->TxUndiBuffer = NULL;
		} else {
			StatFlags |= PXE_STATFLAGS_GET_STATUS_NO_TXBUFS_WRITTEN;
		}
	}

	//Indicate pending interrupt status (in our case indicate any transmit completion and any receive packet exists)
	if (Cdb->OpFlags & PXE_OPFLAGS_GET_INTERRUPT_STATUS) {
		if (Adapter->RxBufferDataPresent) {
			// onlly process the first frame length
			RxCmdA = *(UINT32 *)Adapter->RxBufferDataStart;
			Size = RxCmdA & RX_CMD_A_LEN_MASK_;

			// this size change 11/23/2020 makes 130Kbps to 35Mbps (max performance)
			DbPtr->RxFrameLen = (PXE_UINT32) Size;
			DEBUGPRINT(DBG_UNDI, ("Lan7800UndiPxeGetStatus: RxFrameLen=%d\n", DbPtr->RxFrameLen));
		} else {
			DbPtr->RxFrameLen = 0;
		}
		//if (Adapter->TxUndiBuffer != NULL) {
			//StatFlags |= PXE_STATFLAGS_GET_STATUS_TRANSMIT;
		//}
		StatFlags |= Adapter->UndiDeviceInterruptStatus;
		Adapter->UndiDeviceInterruptStatus = 0;
	}
	
	Cdb->StatFlags = StatFlags | PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
	return;

/*
UndiPxeGetStatusError:
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
	Cdb->StatCode = PXE_STATCODE_DEVICE_FAILURE;
	return;
*/
}

VOID
Lan7800UndiPxeFillHeader (
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	PXE_CPB_FILL_HEADER				*CpbFillHeaderPtr;
	PXE_CPB_FILL_HEADER_FRAGMENTED	*CpbFillHeaderFragPtr;
	ETHER_HEAD						*EtherHeader;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if (Cdb->CPBsize == PXE_CPBSIZE_NOT_USED || Cdb->CPBaddr == PXE_CPBADDR_NOT_USED) {
		goto BadCdb;
	}

	if ((Cdb->OpFlags & PXE_OPFLAGS_FILL_HEADER_OPMASK) == PXE_OPFLAGS_FILL_HEADER_WHOLE) {
		// Fill Header Whole
		if(Cdb->CPBsize != sizeof(PXE_CPB_FILL_HEADER)) {
			goto BadCdb;
		}
		CpbFillHeaderPtr = (PXE_CPB_FILL_HEADER *) (UINTN) Cdb->CPBaddr;
    
		if (CpbFillHeaderPtr->MediaHeaderLen != PXE_MAC_HEADER_LEN_ETHER) {
			goto BadCdb;
		}
      
		EtherHeader = (ETHER_HEAD *)(UINTN) CpbFillHeaderPtr->MediaHeader;
		CopyMem(EtherHeader->DstMac, &CpbFillHeaderPtr->DestAddr, PXE_HWADDR_LEN_ETHER);
		CopyMem(EtherHeader->SrcMac, &CpbFillHeaderPtr->SrcAddr, PXE_HWADDR_LEN_ETHER);
		EtherHeader->EtherType = CpbFillHeaderPtr->Protocol;
	} else {
		// Fill Header Fragmented, //FIXE:not supported
		if(Cdb->CPBsize != sizeof(PXE_CPB_FILL_HEADER_FRAGMENTED)) {
			goto BadCdb;
		}

		CpbFillHeaderFragPtr = (PXE_CPB_FILL_HEADER_FRAGMENTED *) (UINTN) Cdb->CPBaddr;
		if ((CpbFillHeaderFragPtr->MediaHeaderLen != PXE_MAC_HEADER_LEN_ETHER ) || (CpbFillHeaderFragPtr->FragDesc[0].FragLen < PXE_MAC_HEADER_LEN_ETHER)) {
			goto BadCdb;
		}

		EtherHeader = (ETHER_HEAD *)(UINTN)CpbFillHeaderFragPtr->FragDesc[0].FragAddr; 
		CopyMem(EtherHeader->DstMac, &CpbFillHeaderFragPtr->DestAddr, PXE_HWADDR_LEN_ETHER);
		CopyMem(EtherHeader->SrcMac, &CpbFillHeaderFragPtr->SrcAddr, PXE_HWADDR_LEN_ETHER);
		EtherHeader->EtherType = CpbFillHeaderFragPtr->Protocol;
	}
  
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  
	return;

BadCdb:
  Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  Cdb->StatCode = PXE_STATCODE_INVALID_CDB;

  return;
}

VOID
Lan7800UndiPxeTransmit(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	EFI_STATUS					Status = EFI_SUCCESS;
	PXE_CPB_TRANSMIT			*CpbTransmit;
	PXE_CPB_TRANSMIT_FRAGMENTS	*CpbFragTransmit;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if ((Cdb->OpFlags & PXE_OPFLAGS_TRANSMIT_FRAGMENTED) == PXE_OPFLAGS_TRANSMIT_FRAGMENTED) {
		DEBUGPRINT(DBG_UNDI, ("%a: PXE_OPFLAGS_TRANSMIT_FRAGMENTED not supported\n", __FUNCTION__));
		// Fragment packet
		CpbFragTransmit = (PXE_CPB_TRANSMIT_FRAGMENTS *)(UINTN)Cdb->CPBaddr;
		//FIXME:
		//Status = UsbNetFragTransmit( UndiDevice->NetDevice, FragTransmit);
	} else {

		//Current tx buffer transmit is not complete (unless it is indicated to undi in get status)
		if (Adapter->TxUndiBuffer != NULL) {
			goto TransmitError;
		}
		// NonFragmented packet
		CpbTransmit = (PXE_CPB_TRANSMIT *) (UINTN)Cdb->CPBaddr;

		Status = Lan7800Transmit(Adapter, (UINT8 *)(UINTN)CpbTransmit->FrameAddr, 
				CpbTransmit->MediaheaderLen, CpbTransmit->DataLen); 
	}
 
	if (EFI_ERROR(Status)) {
		goto TransmitError;
	}

  Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  Cdb->StatCode = PXE_STATCODE_SUCCESS;

  return;
  
TransmitError:
  Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  Cdb->StatCode = PXE_STATCODE_INVALID_CDB;

  return;
}

VOID
Lan7800UndiPxeReceive(
	LAN7800_ADAPTER_DATA	*Adapter, 
	PXE_CDB					*Cdb
	)
{
	EFI_STATUS			Status;
	PXE_CPB_RECEIVE		*CpbReceive;
	PXE_DB_RECEIVE		*DbReceive;
	UINT8				*CpbBuffer;
	PXE_UINT32			CpbBufferLen;
	PXE_UINT32			FrameLen;
	ETHER_HEAD			*EtherHeader;
  	
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	CpbReceive = (PXE_CPB_RECEIVE *)(UINTN)Cdb->CPBaddr;
	CpbBuffer = (UINT8 *)(UINTN)CpbReceive->BufferAddr;
	CpbBufferLen = CpbReceive->BufferLen;
	DbReceive = (PXE_DB_RECEIVE *)(UINTN)Cdb->DBaddr;
  
	//Receive frame and fill in CpbBuffer
	Status = Lan7800UndiPxeReceiveData(Adapter, CpbBuffer, CpbBufferLen, &FrameLen);
	if (EFI_ERROR(Status)) {
		goto UndiPxeReceiveError;
	}

	if (FrameLen == 0) {
		Cdb->StatCode = PXE_STATCODE_NO_DATA;
		Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
		return;
	}

	EtherHeader = (ETHER_HEAD *)CpbBuffer;
  
	CopyMem(&DbReceive->DestAddr, EtherHeader->DstMac, NET_ETHER_ADDR_LEN);
	CopyMem(&DbReceive->SrcAddr, EtherHeader->SrcMac, NET_ETHER_ADDR_LEN);
	DbReceive->FrameLen = FrameLen;
	DbReceive->Protocol = EtherHeader->EtherType;
	DbReceive->MediaHeaderLen = PXE_MAC_HEADER_LEN_ETHER;

	//Set the frame type
	if (IsBroadcastMacAddress(EtherHeader->DstMac)) {
		DbReceive->Type = PXE_FRAME_TYPE_BROADCAST;
	} else if (CompareMem(EtherHeader->DstMac, Adapter->MacAddress, NET_ETHER_ADDR_LEN) == 0) {
		DbReceive->Type = PXE_FRAME_TYPE_UNICAST;
	} else {
		if (IsMulticaseMacAddress(EtherHeader->DstMac)) {
			DbReceive->Type = PXE_FRAME_TYPE_MULTICAST;
		} else {
			DbReceive->Type = PXE_FRAME_TYPE_PROMISCUOUS;
		}
	}

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
	Cdb->StatCode = PXE_STATCODE_SUCCESS;
  
	return;
  
UndiPxeReceiveError:

	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
	Cdb->StatCode = PXE_STATCODE_INVALID_CPB;
  
	return;
}

EFI_STATUS
Lan7800UndiPxeReceiveData(
	LAN7800_ADAPTER_DATA	*Adapter,
	UINT8					*Buffer,
	PXE_UINT32				BufferLen,
	PXE_UINT32				*FrameLen
	)
{
	UINT32		TempLen;
	UINT16		Size;
	UINT32		RxCmdA;
	UINT32		Alignment;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	// 11/24/2020 add call to Lan7800Receive here.
	if (Adapter->RxTxPathEnabled && (Adapter->RxBufferDataPresent == 0)) {
		Status = Lan7800Receive(Adapter);	// do USB bulktransfer 
	}

	//data present, copy the data to Cpb buffer
	if (Adapter->RxBufferDataPresent) {
		//extract frame
		RxCmdA = *(UINT32 *)Adapter->RxBufferDataStart;
		Size = RxCmdA & RX_CMD_A_LEN_MASK_;
		Alignment = (4 - ((Size + 2) % 4)) % 4;

		if (RxCmdA & RX_CMD_A_RED_) {
			Adapter->RxBufferDataLen = 0;
			Adapter->RxBufferDataPresent = 0;
			return Status;
		}

		//If the Undi (Cpb) buffer is smaller than the actual data, copy only the buffer size
		TempLen = Size;
		if (Size > BufferLen) {
			TempLen = BufferLen;
		}
		//Leave 10 bytes for CmdA + CmdB + CmdC
		CopyMem(Buffer, Adapter->RxBufferDataStart + 10, TempLen);
		//Set the actual data len we received
		*FrameLen = Size;
		
		Adapter->RxBufferDataLen -= (10 + Size);
		if (Adapter->RxBufferDataLen) {
			Adapter->RxBufferDataLen -= Alignment;
			//Set the RxBufferStart to next packet
			Adapter->RxBufferDataStart += (10 + Size + Alignment);
			//Since we have data left in the Rx buffer indicate so
			Adapter->UndiDeviceInterruptStatus |= PXE_STATFLAGS_GET_STATUS_RECEIVE;
		} else {
			Adapter->RxBufferDataPresent = 0;
		}
		//PacketDump(Buffer, Adapter->RxBufferDataLen);
	}

	return Status;
}

EFI_STATUS 
Lan7800UndiDeviceDeInitialize(
	LAN7800_ADAPTER_DATA	*Adapter
	)
{
	return RemoveInterface(Adapter);
}

VOID
Lan7800UndiPxeEntry(
	PXE_CDB     *Cdb
	)
{
	PXE_OPCODE				PxeOpCode;
	LAN7800_ADAPTER_DATA	*Adapter;
	UNDI_CALL_TABLE			*callTablePtr;
	
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if (Cdb == NULL) {
		return;
	}
	//IFnum not larger than mximum instance
	if (Cdb->IFnum > GetInterfaceCount()) {
		goto UndiPxeEntryError;
	}
	//Cdb's OpCode must not larger than PXE_OPCODE_LAST_VALID
	if (Cdb->OpCode > PXE_OPCODE_LAST_VALID) {
		goto UndiPxeEntryError;
	}

	// Cdb's StatCode and StatFlags must be initialized to PXE_STATCODE_INITIALIZE
	if (Cdb->StatCode != PXE_STATCODE_INITIALIZE || Cdb->StatFlags != PXE_STATFLAGS_INITIALIZE) {
		goto UndiPxeEntryError;
	}

	//Get the adapter structure based on IFnum
	Adapter = gLan7800UndiDevices[Cdb->IFnum];
	PxeOpCode = Cdb->OpCode;

	//Get the call table entry
	callTablePtr = &UndiCallTable[PxeOpCode];

	//FIXME: Validate the cable entry to with passed Cdb

	//Call the UNDI function from the call table
	UndiCallTable[PxeOpCode].FunctionPtr(Adapter, Cdb);

	return;

UndiPxeEntryError:
	Cdb->StatCode = PXE_STATCODE_INVALID_CDB;
	Cdb->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    return;
}

VOID
Lan7800PxeSwFixedCheckSum(
	PXE_SW_UNDI		*PxeSwUndi
  )
{
	UINT8	*Data;
	UINTN	Index;
	UINT8	Sum = 0;

	// reset Fudeg field
	PxeSwUndi->Fudge = 0;

	Data = (UINT8 *) PxeSwUndi;

	// accumulate all byte of  PXE_SW_UNDI
	for (Index = 0; Index < sizeof(PXE_SW_UNDI); Index++) {
		Sum = Sum + *Data;
		Data++;
	}

	//CheckSum value
	PxeSwUndi->Fudge = (UINT8)(0x100 - Sum);
  
	return;
}

EFI_STATUS
Lan7800UndiModuleInit(
	EFI_LOADED_IMAGE_PROTOCOL	*LoadedImage
	)
{
	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	gLan7800ImageAddr = (UINT64)(UINTN) LoadedImage->ImageBase;
	gLan7800ImageSize = (UINT32) LoadedImage->ImageSize;

	// Allocate memory for PXE_SW_UNDI structure
	gLan7800PxeSwUndiBuffer = AllocateZeroPool(sizeof(PXE_SW_UNDI) + 0x0f);
	if (gLan7800PxeSwUndiBuffer == NULL) {
		DEBUGPRINT(DBG_ERROR, ("%a:  Allocate PXE_SW_UNDI Buffer Failed\n", __FUNCTION__));
		return EFI_OUT_OF_RESOURCES;
	}

	// PXE_SW_UNDI structure MUST align on 0x10
	gLan7800PxeSwUndi = (PXE_SW_UNDI *)(UINTN)(((UINTN)gLan7800PxeSwUndiBuffer + 0x0f) & 0xfffffffffffffff0ULL);

	// Initial PXE_SW_UNDI Structure
	gLan7800PxeSwUndi->Signature = PXE_BUSTYPE_PXE;
	gLan7800PxeSwUndi->Len = sizeof (PXE_SW_UNDI);
	gLan7800PxeSwUndi->Fudge = 0;
	gLan7800PxeSwUndi->Rev = PXE_ROMID_REV;
	gLan7800PxeSwUndi->IFcnt = 0;
	gLan7800PxeSwUndi->MajorVer = PXE_ROMID_MAJORVER;
	gLan7800PxeSwUndi->MinorVer = PXE_ROMID_MINORVER;
	gLan7800PxeSwUndi->Implementation =  PXE_ROMID_IMP_SW_VIRT_ADDR |
										//PXE_ROMID_IMP_CMD_LINK_SUPPORTED |
										PXE_ROMID_IMP_NVDATA_READ_ONLY |
										PXE_ROMID_IMP_STATION_ADDR_SETTABLE |
										PXE_ROMID_IMP_PROMISCUOUS_MULTICAST_RX_SUPPORTED |
										PXE_ROMID_IMP_PROMISCUOUS_RX_SUPPORTED |
										PXE_ROMID_IMP_BROADCAST_RX_SUPPORTED |
										PXE_ROMID_IMP_FILTERED_MULTICAST_RX_SUPPORTED;
	gLan7800PxeSwUndi->EntryPoint = (PXE_UINT64)(UINTN)Lan7800UndiPxeEntry;
	gLan7800PxeSwUndi->BusCnt = 1;
	gLan7800PxeSwUndi->BusType[0] = PXE_BUSTYPE_PCI;
  
	return EFI_SUCCESS;
}


EFI_STATUS
Lan7800UndiProtocolInitialize(
	VOID	*pAdapterInstance
  )
{
	LAN7800_ADAPTER_DATA	*Adapter;

	DEBUGPRINT(DBG_UNDI, ("%a\n", __FUNCTION__));

	if (pAdapterInstance == NULL)
		return EFI_INVALID_PARAMETER;

	Adapter = (LAN7800_ADAPTER_DATA *)pAdapterInstance;

	// Make adapter UndiState at Stopped
	Adapter->UndiState = UNDI_STATE_STOPPED;

	// Initial NII Protocol
	Adapter->NIIProtocol_31.Revision = LAN7800_NII_VERSION;
	Adapter->NIIProtocol_31.Id = (UINT64) gLan7800PxeSwUndi;
	Adapter->NIIProtocol_31.ImageAddr = gLan7800ImageAddr;
	Adapter->NIIProtocol_31.ImageSize = gLan7800ImageSize;
	Adapter->NIIProtocol_31.StringId[0] = 'U';
	Adapter->NIIProtocol_31.StringId[1] = 'N';
	Adapter->NIIProtocol_31.StringId[2] = 'D';  
	Adapter->NIIProtocol_31.StringId[3] = 'I';  
	Adapter->NIIProtocol_31.Type = EfiNetworkInterfaceUndi;
	Adapter->NIIProtocol_31.MajorVer = PXE_ROMID_MAJORVER;
	Adapter->NIIProtocol_31.MinorVer = PXE_ROMID_MINORVER;
	Adapter->NIIProtocol_31.Ipv6Supported = TRUE;
	Adapter->NIIProtocol_31.IfNum = GetInterfaceNumber(Adapter);

	// Update the adapter count 
	gLan7800PxeSwUndi->IFcnt = GetInterfaceCount();
	Lan7800PxeSwFixedCheckSum(gLan7800PxeSwUndi);

	DEBUGPRINT(DBG_UNDI, ("%a, IfNum=%d\n", __FUNCTION__, Adapter->NIIProtocol_31.IfNum));

  return EFI_SUCCESS;
}
















