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

EFI_STATUS
Lan7800DriverBindingSupported(
	IN EFI_DRIVER_BINDING_PROTOCOL	*This,
	IN EFI_HANDLE					Controller,
	IN EFI_DEVICE_PATH_PROTOCOL		*RemainingDevicePath
  );

EFI_STATUS
Lan7800DriverBindingStart(
	IN EFI_DRIVER_BINDING_PROTOCOL	*This,
	IN EFI_HANDLE					Controller,
	IN EFI_DEVICE_PATH_PROTOCOL		*RemainingDevicePath
  );

EFI_STATUS
Lan7800DriverBindingStop(
	IN  EFI_DRIVER_BINDING_PROTOCOL *This,
	IN  EFI_HANDLE					Controller,
	IN  UINTN                       NumberOfChildren,
	IN  EFI_HANDLE                  *ChildHandleBuffer
);

EFI_STATUS
EFIAPI
Lan7800DriverRunDiagnostics(
	IN EFI_DRIVER_DIAGNOSTICS_PROTOCOL	*This,
	IN EFI_HANDLE						ControllerHandle,
	IN EFI_HANDLE						ChildHandle OPTIONAL,
	IN EFI_DRIVER_DIAGNOSTIC_TYPE		DiagnosticType,
	IN CHAR8							*Language,
	OUT EFI_GUID						**ErrorType,
	OUT UINTN							*BufferSize,
	OUT CHAR16							**Buffer
  );

// EFI Component Name Protocol
GLOBAL_REMOVE_IF_UNREFERENCED EFI_COMPONENT_NAME_PROTOCOL gLan7800ComponentName = {
	Lan7800GetDriverName,
	Lan7800GetControllerName,
	"eng"
};
GLOBAL_REMOVE_IF_UNREFERENCED EFI_COMPONENT_NAME2_PROTOCOL gLan7800ComponentName2 = {
	(EFI_COMPONENT_NAME2_GET_DRIVER_NAME) Lan7800GetDriverName,
	(EFI_COMPONENT_NAME2_GET_CONTROLLER_NAME) Lan7800GetControllerName,
	"en-US"
};
GLOBAL_REMOVE_IF_UNREFERENCED EFI_UNICODE_STRING_TABLE gLan7800DriverNameTable[] = {
	{"eng", LAN7800_DRIVER_NAME},
	{"en-US", LAN7800_DRIVER_NAME},
	{NULL,  NULL}
};

// EFI Driver Diagnostics Protocol
GLOBAL_REMOVE_IF_UNREFERENCED EFI_DRIVER_DIAGNOSTICS_PROTOCOL gLan7800DriverDiagnostics = {
	Lan7800DriverRunDiagnostics,
	"eng"
};
GLOBAL_REMOVE_IF_UNREFERENCED EFI_DRIVER_DIAGNOSTICS2_PROTOCOL gLan7800DriverDiagnostics2 = {
  (EFI_DRIVER_DIAGNOSTICS2_RUN_DIAGNOSTICS) Lan7800DriverRunDiagnostics,
  "en-US"
};

//Image handle, System table
EFI_HANDLE			gLan7800ImageHandle;
EFI_SYSTEM_TABLE	*gLan7800SystemTable;

EFI_DRIVER_BINDING_PROTOCOL gLan7800DriverBinding = {
  Lan7800DriverBindingSupported,
  Lan7800DriverBindingStart,
  Lan7800DriverBindingStop,
  DRIVER_VERSION,
  NULL,
  NULL
};

EFI_STATUS
EFIAPI
Lan7800GetDriverName(
	IN  EFI_COMPONENT_NAME_PROTOCOL	*This,
	IN  CHAR8						*Language,
	OUT CHAR16						**DriverName
	)
{
	if (This == NULL || Language == NULL || DriverName == NULL) {
		return EFI_INVALID_PARAMETER;
	}
  
	return LookupUnicodeString2(Language, This->SupportedLanguages, gLan7800DriverNameTable,
		DriverName, (BOOLEAN)(This == &gLan7800ComponentName));
}

EFI_STATUS
EFIAPI
Lan7800GetControllerName(
	IN  EFI_COMPONENT_NAME_PROTOCOL	*This,
	IN  EFI_HANDLE					ControllerHandle,
	IN  EFI_HANDLE					ChildHandle	OPTIONAL,
	IN  CHAR8						*Language,
	OUT CHAR16						**ControllerName
	)
{
	EFI_STATUS					Status = EFI_SUCCESS;
	LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmdOps;
	EFI_DEVICE_PATH_PROTOCOL	*DevicePath;
	LAN7800_ADAPTER_DATA		*Adapter;

	if ((This == NULL) || (ControllerHandle == NULL) || (Language == NULL) || (ControllerName == NULL)) {
		return EFI_INVALID_PARAMETER;
	}

	// Check child handle
	if (ChildHandle != NULL) {
		Status = gBS->HandleProtocol(ChildHandle, &gLan7800DeviceCmdProtocolGuid, &DeviceCmdOps);
		if (EFI_ERROR(Status)) {
			return EFI_UNSUPPORTED;
		}
	} else {
		return EFI_UNSUPPORTED;
	}

	// Check controller handle
	Status = gBS->OpenProtocol(
			ControllerHandle,
			&gEfiDevicePathProtocolGuid,
			&DevicePath,
			gLan7800DriverBinding.DriverBindingHandle,
			ControllerHandle,
			EFI_OPEN_PROTOCOL_BY_DRIVER
			);

	if (Status != EFI_ALREADY_STARTED) {
		gBS->CloseProtocol(
			ControllerHandle,
			&gEfiDevicePathProtocolGuid,
			gLan7800DriverBinding.DriverBindingHandle,
			ControllerHandle
			);    
		return EFI_UNSUPPORTED; 
	}

	Status = gBS->OpenProtocol(
			ChildHandle,
			&gLan7800DeviceCmdProtocolGuid,
			&DeviceCmdOps,
			gLan7800DriverBinding.DriverBindingHandle,
			ChildHandle,
			EFI_OPEN_PROTOCOL_GET_PROTOCOL
			);
	if (EFI_ERROR(Status)) {
		return EFI_UNSUPPORTED;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(DeviceCmdOps);

	return LookupUnicodeString2(
           Language,
           This->SupportedLanguages,
           Adapter->ControllerNameTable,
           ControllerName,
           (BOOLEAN)(This == &gLan7800ComponentName)
           );	
}

EFI_STATUS 
Lan7800DriverInitialize(
	LAN7800_ADAPTER_DATA *Adapter
	)
{
	EFI_STATUS						Status = EFI_SUCCESS;
	EFI_USB_DEVICE_DESCRIPTOR		DeviceDescriptor;
	EFI_USB_INTERFACE_DESCRIPTOR	InterfaceDescriptor;
	EFI_USB_ENDPOINT_DESCRIPTOR		EndpointDescriptor;
	UINT8							Index;
	EFI_USB_IO_PROTOCOL				*UsbIo = Adapter->UsbIo;

	DEBUGPRINT(DBG_DRV, ("%a\n", __FUNCTION__));

	Status = UsbIo->UsbGetDeviceDescriptor(UsbIo, &DeviceDescriptor);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("UsbGetDeviceDescriptor error: %r\n", Status));
		return EFI_DEVICE_ERROR;
	}
	
	Adapter->IdVendor = DeviceDescriptor.IdVendor;
	Adapter->IdProduct = DeviceDescriptor.IdProduct;
	Adapter->DeviceSpeedCapbility = DeviceDescriptor.BcdUSB;

	//Copy appropriate device name to DeviceName 
	if (Adapter->IdProduct == 0x7800) {
		memcpy(Adapter->DeviceName, L"LAN7800", 14);
	} else if (Adapter->IdProduct == 0x7801) {
		memcpy(Adapter->DeviceName, L"LAN7801", 14);
	} else if (Adapter->IdProduct == 0x7850) {
		memcpy(Adapter->DeviceName, L"LAN7850", 14);
	} else if (Adapter->IdProduct == 0x780a) {
		memcpy(Adapter->DeviceName, L"LAN780A", 14);
	}
	//Get interface descriptor
	Status = UsbIo->UsbGetInterfaceDescriptor(UsbIo, &InterfaceDescriptor);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("UsbGetInterfaceDescriptor error: %r\n", Status));
		return EFI_DEVICE_ERROR;
	}

	//Get endpoint address
	for (Index = 0; Index < InterfaceDescriptor.NumEndpoints; Index++) {
		UsbIo->UsbGetEndpointDescriptor(UsbIo, Index, &EndpointDescriptor);
		switch (EndpointDescriptor.Attributes & USB_ENDPOINT_TYPE_MASK) {
			case USB_ENDPOINT_BULK :
				if ((EndpointDescriptor.EndpointAddress & USB_ENDPOINT_DIR_IN) == USB_ENDPOINT_DIR_IN) {
					Adapter->UsbEndpointInfo.EndpointBulkIn = EndpointDescriptor.EndpointAddress;
					if (EndpointDescriptor.MaxPacketSize < 64) {
						DEBUGPRINT(DBG_DRV, ("EndpointDescriptor.MaxPacketSize = %d\n", EndpointDescriptor.MaxPacketSize));
						return EFI_DEVICE_ERROR;
					}
				} else {
					Adapter->UsbEndpointInfo.EndpointBulkOut = EndpointDescriptor.EndpointAddress & 0x0f;
				} 
				break;

			case USB_ENDPOINT_INTERRUPT :
				Adapter->UsbEndpointInfo.EndpointInterrupt = USB_ENDPOINT_DIR_IN | (EndpointDescriptor.EndpointAddress & 0x0f);
				break;       
			default :
				break;
		}
	}

	// Reset and initialize the device
	Status = Lan7800DeviceInitialize(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Allocate Tx and Rx buffers
	Adapter->TxBuffer = AllocateZeroPool(Adapter->TxBufferSize);
	if (Adapter->TxBuffer == NULL) {
		DEBUGPRINT(DBG_ERROR, ("TxBuffer allocation failed\n"));
		return 	EFI_OUT_OF_RESOURCES;	
	}

	Adapter->RxBuffer = AllocateZeroPool(Adapter->RxBufferSize);
	if (Adapter->RxBuffer == NULL) {
		DEBUGPRINT(DBG_ERROR, ("RxBuffer allocation failed\n"));
		return 	EFI_OUT_OF_RESOURCES;	
	}
	Adapter->RxBufferDataPresent = 0;
	Adapter->UndiDeviceInterruptStatus = 0;
	Adapter->RxBufferDataLen = 0;

	return Status;
}

EFI_STATUS
Lan7800DriverDeInitialize(
	LAN7800_ADAPTER_DATA *Adapter
	)
{
	//Free Tx and Rx buffers
	gBS->FreePool(Adapter->TxBuffer);
	gBS->FreePool(Adapter->RxBuffer);

	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800DriverBindingSupported(
	IN EFI_DRIVER_BINDING_PROTOCOL	*This,
	IN EFI_HANDLE					Controller,
	IN EFI_DEVICE_PATH_PROTOCOL		*RemainingDevicePath
  )
{
	EFI_STATUS                    Status;
	EFI_USB_IO_PROTOCOL           *UsbIo;
	EFI_USB_DEVICE_DESCRIPTOR     DeviceDescriptor;

	DEBUGPRINT(DBG_DRV, ("%a\n", __FUNCTION__));

	// Get the USB IO protocol for the controlelr
	Status = gBS->OpenProtocol(
                  Controller,
                  &gEfiUsbIoProtocolGuid,
                  &UsbIo,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
	if (EFI_ERROR(Status)) {
		return Status;
	}

	// Get VID and PID
	Status = UsbIo->UsbGetDeviceDescriptor(UsbIo, &DeviceDescriptor);
	if (EFI_ERROR(Status)) {
		goto DONE;
	}

	// Check device IDs
	if ((DeviceDescriptor.IdVendor == 0x0424) && 
		((DeviceDescriptor.IdProduct == 0x7800) || 
		(DeviceDescriptor.IdProduct == 0x7801) ||
		(DeviceDescriptor.IdProduct == 0x7850) ||
		(DeviceDescriptor.IdProduct == 0x780A))) {
		Status = EFI_SUCCESS;
	} else {
		Status = EFI_UNSUPPORTED;
	}

DONE:
	gBS->CloseProtocol(
         Controller,
         &gEfiUsbIoProtocolGuid,
         This->DriverBindingHandle,
         Controller
         );

	return Status;
}


EFI_STATUS
Lan7800DriverBindingStart (
  IN EFI_DRIVER_BINDING_PROTOCOL	*This,
  IN EFI_HANDLE						Controller,
  IN EFI_DEVICE_PATH_PROTOCOL		*RemainingDevicePath
  )
{
	EFI_STATUS						Status;
	EFI_USB_IO_PROTOCOL				*UsbIo;
	LAN7800_ADAPTER_DATA			*Adapter = NULL;
	EFI_DEVICE_PATH_PROTOCOL		*ParentDevicePath;
	MAC_ADDRESS_DEVICE_PATH_NODE	TempMacAddrPath;

	DEBUGPRINT(DBG_DRV, ("%a\n", __FUNCTION__));

	// Get the device path for the handle
	ParentDevicePath = NULL;
	Status = gBS->OpenProtocol(
                  Controller,
                  &gEfiDevicePathProtocolGuid,
                  &ParentDevicePath,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
                  
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("Open DevicePath Protocol on Controller: %r\n", Status));
		return Status;
	}

	// Get the UsbIo 
	Status = gBS->OpenProtocol(
                  Controller,
                  &gEfiUsbIoProtocolGuid,
                  &UsbIo,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
	if (EFI_ERROR (Status)) {
		DEBUGPRINT(DBG_ERROR, ("Open UsbIo Protocol on Controller: %r\n", Status));
		goto DONE1;
	}

	//Allocate Adpater structure
	Adapter = AllocateZeroPool(sizeof(LAN7800_ADAPTER_DATA));
	if (Adapter == NULL) {
		DEBUGPRINT(DBG_ERROR, ("Adapter allocation failed\n"));
		goto DONE;
	}

	Adapter->Signature = LAN7800_ADAPTER_SIGNATURE;
	Adapter->ControllerHandle = Controller;
	Adapter->UsbIo = UsbIo;

	//Initialize the device
	Status = Lan7800DriverInitialize(Adapter);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_DRV, ("Lan7800DriverInitialize failed : %r\n", Status));
		goto DONE;
	}

	//Create the device path for the device using device's (unique) MAC address
	TempMacAddrPath.MacNode.Header.Type = MESSAGING_DEVICE_PATH;
	TempMacAddrPath.MacNode.Header.SubType = MSG_MAC_ADDR_DP;
	TempMacAddrPath.MacNode.Header.Length[0] = sizeof(MAC_ADDR_DEVICE_PATH);
	TempMacAddrPath.MacNode.Header.Length[1] = 0;
	CopyMem(&TempMacAddrPath.MacNode.MacAddress, Adapter->MacAddress, sizeof(EFI_MAC_ADDRESS));
	TempMacAddrPath.EndNode.Type = END_DEVICE_PATH_TYPE;
	TempMacAddrPath.EndNode.SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE;
	TempMacAddrPath.EndNode.Length[0] = END_DEVICE_PATH_LENGTH;
	TempMacAddrPath.EndNode.Length[1] = 0;
	Adapter->DevicePath = AppendDevicePath(ParentDevicePath, (EFI_DEVICE_PATH_PROTOCOL *)&TempMacAddrPath);

	//Initialize device command protocol
	Adapter->DeviceCmdOps.GetDeviceName = Lan7800GetDeviceNameCmd;
	Adapter->DeviceCmdOps.GetMacAddress = Lan7800GetMacAddressCmd;
	Adapter->DeviceCmdOps.GetEepromLength = Lan7800GetEepromLengthCmd;
	Adapter->DeviceCmdOps.ReadEeprom = Lan7800ReadEepromCmd;
	Adapter->DeviceCmdOps.WriteEeprom = Lan7800WriteEepromCmd;
	Adapter->DeviceCmdOps.ReadPhy = Lan7800ReadPhyCmd;
	Adapter->DeviceCmdOps.WritePhy = Lan7800WritePhyCmd;
	Adapter->DeviceCmdOps.ReadRegister = Lan7800ReadRegisterCmd;
	Adapter->DeviceCmdOps.WriteRegister = Lan7800WriteRegisterCmd;
	Adapter->DeviceCmdOps.DumpOtp = Lan7800DumpOtpCmd;
	Adapter->DeviceCmdOps.GetLinkStatus = Lan7800GetLinkStatusCmd;

	//Initialize NIIProtocol parapeters
	Lan7800UndiProtocolInitialize(Adapter); //Lan7800Undi.c

	// Install Device Path Protocol in the new handle
	Status = gBS->InstallMultipleProtocolInterfaces(
				&Adapter->DeviceHandle,
				&gEfiDevicePathProtocolGuid,
				Adapter->DevicePath,
				&gLan7800DeviceCmdProtocolGuid,
				&Adapter->DeviceCmdOps,
				&gEfiNetworkInterfaceIdentifierProtocolGuid_31,
				&Adapter->NIIProtocol_31,
				NULL
				);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("InstallMultipleProtocolInterfaces failed: %r\n", Status));
		goto DONE;
	}
	
	//Create parent child relationship
	Status = gBS->OpenProtocol(
				Controller,
				&gEfiUsbIoProtocolGuid,
				&UsbIo,
				This->DriverBindingHandle,
				Adapter->DeviceHandle,
				EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER
				);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER: %r\n", Status));
		goto DONE;
	}

	// Add name to ComponentNameTable
	AddUnicodeString2(
		"eng",
		gLan7800ComponentName.SupportedLanguages,
		&Adapter->ControllerNameTable,
		Adapter->DeviceName,
		TRUE
		);
  
	AddUnicodeString2(
		"en-US",
		gLan7800ComponentName2.SupportedLanguages,
		&Adapter->ControllerNameTable,
		Adapter->DeviceName,
		FALSE
		);

	return EFI_SUCCESS;

DONE:
	gBS->CloseProtocol(
		Controller,
		&gEfiUsbIoProtocolGuid,
		This->DriverBindingHandle,
		Controller
		);

DONE1:
	gBS->CloseProtocol (
		Controller,
		&gEfiDevicePathProtocolGuid,
		This->DriverBindingHandle,
		Controller
		);

	if (Adapter != NULL) {
		if (Adapter->DevicePath != NULL) {
			FreePool(Adapter->DevicePath);
		}
		FreePool(Adapter);
	}
  return Status;
}

EFI_STATUS
Lan7800DriverBindingStop (
	IN  EFI_DRIVER_BINDING_PROTOCOL *This,
	IN  EFI_HANDLE					Controller,
	IN  UINTN                       NumberOfChildren,
	IN  EFI_HANDLE                  *ChildHandleBuffer
)
{
	EFI_STATUS					Status = EFI_SUCCESS;
	LAN7800_ADAPTER_DATA		*Adapter;
	LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmdOps;

	DEBUGPRINT(DBG_DRV, ("%a\n", __FUNCTION__));
  
	// Child handle number shouldn't larger then 1
	if (NumberOfChildren > 1) {
		return EFI_INVALID_PARAMETER;
	}
  
	// If we are called with less than one child handle it means that we already sucessfully
	// uninstalled
	if (NumberOfChildren == 0) {
		Status = gBS->CloseProtocol (
					Controller,
					&gEfiDevicePathProtocolGuid,
					This->DriverBindingHandle,
					Controller
					);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		Status = gBS->CloseProtocol (
					Controller,
					&gEfiUsbIoProtocolGuid,
					This->DriverBindingHandle,
					Controller
					);
		if (EFI_ERROR (Status)) {
			return Status;
		}

		return EFI_SUCCESS;
	}
  
	// Using device path to get private data
	Status = gBS->OpenProtocol(
				ChildHandleBuffer[0],
				&gLan7800DeviceCmdProtocolGuid,
				&DeviceCmdOps,
				This->DriverBindingHandle,
				Controller,
				EFI_OPEN_PROTOCOL_GET_PROTOCOL
				);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(DeviceCmdOps);

	// Close bus protocol
	Status = gBS->CloseProtocol (
				Controller,
				&gEfiUsbIoProtocolGuid,
				This->DriverBindingHandle,
				ChildHandleBuffer[0]
				);
	if (EFI_ERROR (Status)) {
		return Status;
	}
  
	Status = gBS->UninstallMultipleProtocolInterfaces (
				Adapter->DeviceHandle,
				&gLan7800DeviceCmdProtocolGuid,
				&Adapter->DeviceCmdOps,
				&gEfiDevicePathProtocolGuid,
				Adapter->DevicePath,
				&gEfiNetworkInterfaceIdentifierProtocolGuid_31,
				&Adapter->NIIProtocol_31,
				NULL
				);
	if (Status == EFI_ACCESS_DENIED) {
		DEBUGPRINT(DBG_ERROR, ("UninstallMultipleProtocolInterfaces access denied, trying again: %r\n", Status));
		Status = gBS->UninstallMultipleProtocolInterfaces (
                    Adapter->DeviceHandle,
                    &gLan7800DeviceCmdProtocolGuid,
                    &Adapter->DeviceCmdOps,
                    &gEfiDevicePathProtocolGuid,
                    Adapter->DevicePath,
                    &gEfiNetworkInterfaceIdentifierProtocolGuid_31,
                    &Adapter->NIIProtocol_31,
                    NULL
                    );
	}

	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Free driver (Tx/Rx resorces)
	Lan7800DriverDeInitialize(Adapter);

	//Free Undi device instance
	Lan7800UndiDeviceDeInitialize(Adapter);
  
	if (Adapter->ControllerNameTable) {
		FreeUnicodeStringTable(Adapter->ControllerNameTable);
	}

	FreePool(Adapter->DevicePath);
	FreePool(Adapter);

	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800DriverUnload(
	IN EFI_HANDLE  ImageHandle
)
{
	EFI_STATUS	Status = EFI_SUCCESS;
	EFI_HANDLE	*DeviceHandleBuffer;
	UINTN		DeviceHandleCount;
	UINTN		Index;

	DEBUGPRINT(DBG_DRV,("%a\n", __FUNCTION__));

	Status = gBS->LocateHandleBuffer(
				AllHandles,
				NULL,
				NULL,
				&DeviceHandleCount,
				&DeviceHandleBuffer
				);

	if (!EFI_ERROR (Status)) {
		for (Index = 0; Index < DeviceHandleCount; Index++) {
			Status = gBS->DisconnectController(
						DeviceHandleBuffer[Index],
						ImageHandle,
						NULL
						);
		}
		if (DeviceHandleBuffer != NULL) {
			gBS->FreePool(DeviceHandleBuffer);
		}
	}

	Status = gBS->UninstallMultipleProtocolInterfaces(
				ImageHandle,
				&gEfiDriverBindingProtocolGuid,
				&gLan7800DriverBinding,
				&gEfiComponentNameProtocolGuid,
				&gLan7800ComponentName,
				&gEfiComponentName2ProtocolGuid,
				&gLan7800ComponentName2,
				NULL
				);

	return Status;
}

EFI_STATUS
Lan7800DriverEntryPoint(
	IN EFI_HANDLE		ImageHandle,
	IN EFI_SYSTEM_TABLE	*SystemTable
	)
{
	EFI_STATUS                 Status;
	EFI_LOADED_IMAGE_PROTOCOL  *LoadedImage;

	DEBUGPRINT(DBG_DRV,("%a\n", __FUNCTION__));
  
	gLan7800ImageHandle = ImageHandle;
	gLan7800SystemTable = SystemTable;

	// Install driver binding protocol	
	Status = EfiLibInstallAllDriverProtocols2(
            ImageHandle,
            SystemTable,
            &gLan7800DriverBinding,
            ImageHandle,
            &gLan7800ComponentName,
            &gLan7800ComponentName2,
            NULL,
            NULL,
            &gLan7800DriverDiagnostics, 
            &gLan7800DriverDiagnostics2
            );

	if (EFI_ERROR(Status)) {
		return Status;
	}

	// Fill in the Unload() function
	Status = gBS->OpenProtocol(
			ImageHandle,
			&gEfiLoadedImageProtocolGuid,
			&LoadedImage,
			ImageHandle,
			ImageHandle,
			EFI_OPEN_PROTOCOL_GET_PROTOCOL
			);

	if (EFI_ERROR (Status)) {
		return Status;
	}

	LoadedImage->Unload = Lan7800DriverUnload;
	Status = Lan7800UndiModuleInit(LoadedImage);
  
	return Status;
}

CHAR16 * 
Lan7800GetDeviceNameCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;

	if (!This) {
		return NULL;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);

	return Adapter->DeviceName;
}

EFI_STATUS 
Lan7800GetMacAddressCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT EFI_MAC_ADDRESS				*MacAddress
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;

	if ((!This) || (!MacAddress)) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	CopyMem(MacAddress, Adapter->MacAddress, sizeof(PXE_MAC_ADDR));
	return 0;
}

EFI_STATUS
Lan7800GetEepromLengthCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT UINT32						*Length
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if ((!This) || (!Length)) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800GetEepromLength(Adapter, Length);
	return Status;
}

EFI_STATUS
Lan7800ReadEepromCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN	UINTN						Offset,
	IN	UINTN						DataLength,
	OUT	UINT8						*Data
  )
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if ((!This) || (!DataLength)) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800ReadEeprom(Adapter, Offset, DataLength, Data);
	return Status;
}

EFI_STATUS
Lan7800WriteEepromCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINTN						Offset,
	IN UINTN						DataLength,
	IN UINT8						*Data
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if ((!This) || (!DataLength)) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800WriteEeprom(Adapter, Offset, DataLength, Data);
	return Status;
}

EFI_STATUS
Lan7800ReadPhyCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT8						PhyId,
	IN UINT32						Index,
	OUT UINT32						*Data
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800ReadPhyRegister(Adapter, PhyId, Index, Data);
	return Status;
}

EFI_STATUS
Lan7800WritePhyCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT8						PhyId,
	IN UINT32						Index,
	IN UINT32						Data
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800WritePhyRegister(Adapter, PhyId, Index, Data);
	return Status;
}

EFI_STATUS
Lan7800ReadRegisterCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN UINT16						Index,
	OUT UINT32						*Data
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800ReadRegister(Adapter, Index, Data);
	return Status;
}

EFI_STATUS
Lan7800WriteRegisterCmd(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*This,
 	IN UINT16						Index,
	OUT UINT32						Data
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800WriteRegister(Adapter, Index, Data);
	return Status;
}

EFI_STATUS
Lan7800DumpOtpCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	IN	UINTN						Offset,
	IN	UINTN						DataLength,
	OUT	UINT8						*Data
  )
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800ReadOtpInternal(Adapter, Offset, DataLength, Data);
	return Status;
}

EFI_STATUS
Lan7800GetLinkStatusCmd(
	IN	LAN7800_DEVICE_CMD_PROTOCOL	*This,
	OUT	UINT32						*LinkSpeed,
	OUT	UINT32						*Duplex
	)
{
	LAN7800_ADAPTER_DATA	*Adapter;
	EFI_STATUS				Status;

	if (!This) {
		return EFI_INVALID_PARAMETER;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);
	Status = Lan7800GetLinkStatus(Adapter, LinkSpeed, Duplex);
	return Status;
}

EFI_STATUS
Lan7800DriverRunDiagnostics(
	IN EFI_DRIVER_DIAGNOSTICS_PROTOCOL	*This,
	IN EFI_HANDLE						ControllerHandle,
	IN EFI_HANDLE						ChildHandle OPTIONAL,
	IN EFI_DRIVER_DIAGNOSTIC_TYPE		DiagnosticType,
	IN CHAR8							*Language,
	OUT EFI_GUID						**ErrorType,
	OUT UINTN							*BufferSize,
	OUT CHAR16							**Buffer
  )
{
	EFI_STATUS						Status;
	LAN7800_DEVICE_CMD_PROTOCOL		*DeviceCmds;
	EFI_DEVICE_PATH_PROTOCOL		*DevicePath;
	LAN7800_ADAPTER_DATA			*Adapter;

	if ((This == NULL) || 
		(ControllerHandle == NULL) ||
		(Language == NULL) ||
		(ErrorType == NULL) ||
		(BufferSize == NULL) ||
		(Buffer == NULL)) {
			return EFI_INVALID_PARAMETER;
	}

	// Language check
	if (AsciiStrnCmp(Language, This->SupportedLanguages, AsciiStrLen(This->SupportedLanguages)) != 0) {
		return EFI_UNSUPPORTED;
	}

	// Check child handle
	if (ChildHandle != NULL) {
		Status = gBS->HandleProtocol(
					ChildHandle,
					&gLan7800DeviceCmdProtocolGuid,
					&DeviceCmds
					);
		if (EFI_ERROR(Status)) {
			return EFI_UNSUPPORTED;
		}
	}

	// Check controller handle
	Status = gBS->OpenProtocol(
				ControllerHandle,
				&gEfiDevicePathProtocolGuid,
				&DevicePath,
				gLan7800DriverBinding.DriverBindingHandle,
				ControllerHandle,
				EFI_OPEN_PROTOCOL_BY_DRIVER
				);

  if (Status != EFI_ALREADY_STARTED) {
	gBS->CloseProtocol(
           ControllerHandle,
           &gEfiDevicePathProtocolGuid,
           gLan7800DriverBinding.DriverBindingHandle,
           ControllerHandle
           ); 
		return EFI_UNSUPPORTED;
	}

	Status = gBS->OpenProtocol(
				ChildHandle,
				&gLan7800DeviceCmdProtocolGuid,
				&DeviceCmds,
				gLan7800DriverBinding.DriverBindingHandle,
				ChildHandle,
				EFI_OPEN_PROTOCOL_GET_PROTOCOL
				);
	if (EFI_ERROR(Status)) {
		return EFI_UNSUPPORTED;
	}

	Adapter = LAN7800_ADAPTER_DATA_FROM_DEVICE_CMD_OPS(This);

	switch (DiagnosticType) {
	case EfiDriverDiagnosticTypeStandard:
		Status = Lan7800StandardDiagnostics(Adapter);
		break;

	case EfiDriverDiagnosticTypeExtended:
		Status = Lan7800ExtendedDiagnostics(Adapter);
		break;

	case EfiDriverDiagnosticTypeManufacturing:
		Status = Lan7800ManufacturingDiagnostics(Adapter);
		break;

	default:
		Status = EFI_UNSUPPORTED;
		break;
	}

	return Status;
}