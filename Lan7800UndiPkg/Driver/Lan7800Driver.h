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

#ifndef _LAN7800_DRIVER_H_
#define _LAN7800_DRIVER_H_

#include <Uefi.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DebugLib.h>
#include <Library/NetLib.h>
#include <Library/IoLib.h>
#include <Protocol/PciIo.h>
#include <Protocol/UsbIo.h>
#include <Protocol/DevicePath.h>
#include <Protocol/LoadedImage.h>
#include <Protocol/NetworkInterfaceIdentifier.h>
#include <Uefi/UefiPxe.h>
#include <Base.h>
#include <Protocol/Lan7800CmdProtocol.h>

#define DRIVER_VERSION				0x00000006		//Update this for each revision of the driver

//Driver name
#define LAN7800_DRIVER_NAME			L"LAN7800 USB-to-Ehternet Driver"

//Maximum Adapters Supported
#define LAN7800_MAX_DEVICES			16
//Max EEPROM size
#define LAN7800_MAX_EEPROM_LEN		512

//Adapter Signature
#define LAN7800_ADAPTER_SIGNATURE	SIGNATURE_32('7','8','0','0')

typedef struct _MAC_ADDRESS_DEVICE_PATH_NODE {
	MAC_ADDR_DEVICE_PATH		MacNode;
	EFI_DEVICE_PATH_PROTOCOL	EndNode;  
} MAC_ADDRESS_DEVICE_PATH_NODE;

//EFI component name prototypes
EFI_STATUS EFIAPI Lan7800GetDriverName(
	IN EFI_COMPONENT_NAME_PROTOCOL *This, 
	IN CHAR8 *Language, 
	OUT CHAR16 **DriverName);

EFI_STATUS EFIAPI Lan7800GetControllerName(
	IN EFI_COMPONENT_NAME_PROTOCOL *This,
	IN EFI_HANDLE ControllerHandle,
	IN EFI_HANDLE ChildHandle OPTIONAL,
	IN CHAR8 *Language,
	OUT CHAR16 **ControllerName);

typedef PXE_VOID (*PXE31_DELAY)(UINT64 UniqueId, UINTN MicroSeconds);
typedef PXE_VOID (*PXE31_BLOCK)(UINT64 UniqueId, UINT32 Enable);
typedef PXE_VOID (*PXE31_VIRT2PHYS)(UINT64 UniqueId, UINT64 Virtual, UINT64 PhysicalPtr);
typedef PXE_VOID (*PXE31_MEM_IO)(UINT64 UniqueId, UINT8 AccessType, UINT8 Length, UINT64 Port, UINT64 BufferPtr);
typedef PXE_VOID (*PXE31_MAP_MEM)(UINT64 UniqueId, UINT64 Virtual, UINT32 Size, UINT32 Direction, UINT64 PhysicalPtr);
typedef PXE_VOID (*PXE31_UNMAP_MEM)(UINT64 UniqueId, UINT64 Virtual, UINT32 Size, UINT32 Direction, UINT64 PhysicalPtr);
typedef PXE_VOID (*PXE31_SYNC_MEM)(UINT64 UniqueId, UINT64 Virtual, UINT32 Size, UINT32 Direction, UINT64 PhysicalPtr);

typedef struct {
	UINT8	EndpointControl;
	UINT8	EndpointBulkIn;
	UINT8	EndpointBulkOut;
	UINT8	EndpointInterrupt;
} DEVICE_ENDPOINT_INFO;

// Private device Data Structure
typedef struct _LAN7800_ADAPTER_DATA {
	UINTN								Signature;
	EFI_HANDLE							ControllerHandle;
	EFI_HANDLE							DeviceHandle;
	EFI_UNICODE_STRING_TABLE			*ControllerNameTable;
	EFI_DEVICE_PATH_PROTOCOL			*DevicePath;
	LAN7800_DEVICE_CMD_PROTOCOL			DeviceCmdOps;
	EFI_NETWORK_INTERFACE_IDENTIFIER_PROTOCOL	NIIProtocol_31;

	//protocol handle for this adapter
	EFI_USB_IO_PROTOCOL					*UsbIo;
	//usb
	DEVICE_ENDPOINT_INFO				UsbEndpointInfo;

	//PCI controller info for the parent controller, just to satisfy PXE_DB_GET_CONFIG_INFO
	PXE_PCI_CONFIG_INFO					PciConfig;
	//Device info
	UINT16								IdVendor;
	UINT16								IdProduct;
	UINT16								DeviceSpeedCapbility;
	CHAR16								DeviceName[30];
	UINT8								PhyId;
	UINT32								ExternalPhy;
	//Station Address
	PXE_MAC_ADDR						MacAddress;
	//Permanent Address
	PXE_MAC_ADDR						PermanentMacAddress;
	//Broadcast address
	PXE_MAC_ADDR						BroadcastMacAddress;
	//Multicast list
	PXE_MAC_ADDR						MultiCastList[MAX_MCAST_ADDRESS_CNT];
	//Number of Multicast addresses present in the list
	UINT8								MulticastListCount;
	//Receive filter flags (unicast, broadcast, promiscuous, multicast or all multicast)
	PXE_STATFLAGS						ReceiveFilterFlags;

	//Undi
	UINT8								UndiState;
	UINT8								UndiDeviceInterruptStatus;
	UINT8								RxTxPathEnabled;
	//Transmit buffer
	UINT16								TxBufferSize;
	UINT8								*TxBuffer;
	UINT8								*TxFragmentBuffer;
	VOID								*TxUndiBuffer;
	//Receive buffer
	UINT8								*RxBuffer;
	UINT16								RxBufferSize;
	//Rx data present in RxBuffer or empty
	UINT8								RxBufferDataPresent;
	//When data preset (RxBufferDataPresent is set) length of receivced data
	UINTN								RxBufferDataLen;
	//Data start pointer in the RxBuffer
	UINT8								*RxBufferDataStart;
	
	// PXE_31 Initialize functions
	PXE31_DELAY							Delay;
	PXE31_BLOCK							Block;
	PXE31_VIRT2PHYS						Vir2Phys;
	PXE31_MEM_IO						MemIo;
	PXE31_MAP_MEM						MapMem;
	PXE31_UNMAP_MEM						UnMapMem;
	PXE31_SYNC_MEM						SyncMem;
	UINT64								UniqueId;

} LAN7800_ADAPTER_DATA;

CHAR16 * 
Lan7800GetDeviceNameCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This
	);

EFI_STATUS 
Lan7800GetMacAddressCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT EFI_MAC_ADDRESS				*MacAddress
	);

EFI_STATUS
Lan7800GetEepromLengthCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT UINT32						*Length
	);

EFI_STATUS
Lan7800ReadEepromCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN	UINTN						Offset,
	IN	UINTN						DataLength,
	OUT	UINT8						*Data
  );

EFI_STATUS
Lan7800WriteEepromCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINTN						Offset,
	IN UINTN						DataLength,
	IN UINT8						*Data
	);

EFI_STATUS
Lan7800ReadPhyCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT8						PhyId,
	IN UINT32						Index,
	OUT UINT32						*Data
	);

EFI_STATUS
Lan7800WritePhyCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT8						PhyId,
	IN UINT32						Index,
	IN UINT32						Data
	);

EFI_STATUS
Lan7800ReadRegisterCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT16						Index,
	OUT UINT32						*Data
	);

EFI_STATUS
Lan7800WriteRegisterCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
 	IN UINT16						Index,
	OUT UINT32						Data
	);

EFI_STATUS
Lan7800DumpOtpCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN	UINTN						Offset,
	IN	UINTN						DataLength,
	OUT	UINT8						*Data
	);

EFI_STATUS
Lan7800GetLinkStatusCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT	UINT32						*LinkSpeed,
	OUT	UINT32						*Duplex
	);

#endif /* _LAN7800_DRIVER_H_ */

