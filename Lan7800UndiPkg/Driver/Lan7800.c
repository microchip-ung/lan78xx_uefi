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
#include "Lan7800.h"
#include "Debug.h"

EFI_STATUS
Lan7800ReadRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT16				Address,
	OUT UINT32				*Data
	)
{
	EFI_USB_IO_PROTOCOL		*UsbIo;
	EFI_USB_DEVICE_REQUEST	Request;
	UINT32					UsbStatus;
	UINTN					Retry;
	EFI_STATUS				Status = EFI_SUCCESS;

	//DEBUGPRINT(DBG_LAN,("%a\n", __FUNCTION__));

	UsbIo = Adapter->UsbIo;
  
	for (Retry = 0; Retry < 3; Retry++) {
		Request.RequestType = USB_REQ_TYPE_VENDOR | USB_TARGET_DEVICE | USB_ENDPOINT_DIR_IN;
		Request.Request = USB_VENDOR_REQUEST_READ_REGISTER;
		Request.Value = 0;
		Request.Index = Address;
		Request.Length = 4;
      
		Status = UsbIo->UsbControlTransfer(
					UsbIo,
					&Request,
					EfiUsbDataIn,
//					100, //timeout in milisec
					10, //timeout in milisec for better performance
					Data,
					4,  //data length
					&UsbStatus
					);
    
		if (!EFI_ERROR(Status)) {
			return EFI_SUCCESS;
		}
		gBS->Stall(1000);
	}

	return Status;
}


EFI_STATUS
Lan7800WriteRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT16				Address,
	OUT UINT32				Data
  )
{
	EFI_USB_IO_PROTOCOL		*UsbIo;
	EFI_USB_DEVICE_REQUEST	Request;
	UINT32					UsbStatus;
	UINTN					Retry;
	EFI_STATUS				Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	UsbIo = Adapter->UsbIo;

	for (Retry = 0; Retry < 3; ++Retry) {
		Request.RequestType = USB_REQ_TYPE_VENDOR | USB_TARGET_DEVICE;
		Request.Request = USB_VENDOR_REQUEST_WRITE_REGISTER;
		Request.Value = 0;
		Request.Index = Address;
		Request.Length = 4;
        
		Status =  UsbIo->UsbControlTransfer(
					UsbIo,
					&Request,
					EfiUsbDataOut,
//					100, //timeout in milisec
					10, //timeout in milisec for better performance
					&Data,
					4, // data length
					&UsbStatus
					);
		if (!EFI_ERROR (Status)) {
			return EFI_SUCCESS;
		}
		gBS->Stall(1000);
	}

	return Status;
}

EFI_STATUS
Lan7800PhyWaitNotBusy(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		Data;
	UINTN		Timeout = 0;
	EFI_STATUS	Status = EFI_SUCCESS;

	while (Timeout < 3) {
		Status = Lan7800ReadRegister(Adapter, MII_ACC, &Data);
		if (!EFI_ERROR(Status) && !(Data & MII_ACC_MII_BUSY_)) {
			return EFI_SUCCESS;
		}
		gBS->Stall(100);
		Timeout++;
	}

	return EFI_TIMEOUT;
}

#define MII_ACC_READ(id, index)         \
                (((id << MII_ACC_PHY_ADDR_SHIFT_) & MII_ACC_PHY_ADDR_MASK_) \
                | ((index << MII_ACC_MIIRINDA_SHIFT_) & MII_ACC_MIIRINDA_MASK_) \
                | MII_ACC_MII_READ_ | MII_ACC_MII_BUSY_)
#define MII_ACC_WRITE(id, index)                \
                (((id << MII_ACC_PHY_ADDR_SHIFT_) & MII_ACC_PHY_ADDR_MASK_) \
                | ((index << MII_ACC_MIIRINDA_SHIFT_) & MII_ACC_MIIRINDA_MASK_) \
                | MII_ACC_MII_WRITE_ | MII_ACC_MII_BUSY_)

EFI_STATUS
Lan7800ReadPhyRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				PhyId,
	IN UINT32				Index,
	OUT UINT32				*Data
	)
{
	UINT32		Address;
	UINT32		Value;
	EFI_STATUS	Status = EFI_SUCCESS;

	//DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800PhyWaitNotBusy(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	PhyId &= 0x1F;
	Index &= 0x1F;
	Address = MII_ACC_READ(PhyId, Index);
	Status = Lan7800WriteRegister(Adapter, MII_ACC, Address);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Status = Lan7800PhyWaitNotBusy(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Status = Lan7800ReadRegister(Adapter, MII_DATA, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	*Data = (UINT32) (Value & 0xFFFF);

	if (*Data != 0x79ed)
		DEBUGPRINT(DBG_2021, ("%a index %d data 0x%x\n", __FUNCTION__, Index, (UINT32) (Value & 0xFFFF)));

	return EFI_SUCCESS; 
}

EFI_STATUS
Lan7800WritePhyRegister (
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				PhyId,
	IN UINT32				Index,
	OUT UINT32				Data
	)
{
	UINT32		Address;
	EFI_STATUS	Status = EFI_SUCCESS;

	//DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800PhyWaitNotBusy(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Status = Lan7800WriteRegister(Adapter, MII_DATA, Data);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	PhyId &= 0x1F;
	Index &= 0x1F;
	Address = MII_ACC_WRITE(PhyId, Index);
	Status = Lan7800WriteRegister(Adapter, MII_ACC, Address);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Lan7800PhyWaitNotBusy(Adapter);
}

EFI_STATUS
Lan7800WaitEeprom(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		Data = 0;
	UINTN		Timeout = 0;
	EFI_STATUS	Status = EFI_SUCCESS;

	while (Timeout < 3) {
		Status = Lan7800ReadRegister(Adapter, E2P_CMD, &Data);
		if (!EFI_ERROR(Status) && (!(Data & E2P_CMD_EPC_BUSY_) || (Data & E2P_CMD_EPC_TIMEOUT_))) {
			break;
		}

		gBS->Stall (100);
		Timeout++;
	}

	if (Data & (E2P_CMD_EPC_BUSY_ | E2P_CMD_EPC_TIMEOUT_)) {
		return EFI_TIMEOUT;
	}

	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800EepromWaitNotBusy(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		Data;
	UINTN		Timeout = 0;
	EFI_STATUS	Status = EFI_SUCCESS;

	while (Timeout < 3) {
		Status = Lan7800ReadRegister(Adapter, E2P_CMD, &Data);
		if (!EFI_ERROR(Status) && !(Data & E2P_CMD_EPC_BUSY_)) {
			return EFI_SUCCESS;
		}
		gBS->Stall (100);
		Timeout++;
	}

	return EFI_TIMEOUT;
}

EFI_STATUS
Lan7800ReadEeprom(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	)
{
	UINT32		Value;
	UINTN		Index;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800EepromWaitNotBusy(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	for (Index = 0; Index < Length; Index++) {
		Value = E2P_CMD_EPC_BUSY_ | E2P_CMD_EPC_CMD_READ_ | (Offset & E2P_CMD_EPC_ADDR_MASK_);
		Status = Lan7800WriteRegister(Adapter, E2P_CMD, Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		Status = Lan7800WaitEeprom(Adapter);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		Status = Lan7800ReadRegister(Adapter, E2P_DATA, &Value);
		if (EFI_ERROR (Status)) {
			return Status;
		}
    
		Data[Index] = (UINT8)(Value & 0xFF);
		Offset++;
  }

  return EFI_SUCCESS; 
}

EFI_STATUS
Lan7800WriteEeprom(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	)
{
	UINT32		Value;
	UINTN		Index;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800EepromWaitNotBusy(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Value = E2P_CMD_EPC_BUSY_ | E2P_CMD_EPC_CMD_EWEN_;
	Status = Lan7800WriteRegister(Adapter, E2P_CMD, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WaitEeprom(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}
  
	for (Index = 0; Index < Length; Index++) {
		//Fill in data
		Value = Data[Index];
		Status = Lan7800WriteRegister(Adapter, E2P_DATA, Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		//Send Write command
		Value = E2P_CMD_EPC_BUSY_ | E2P_CMD_EPC_CMD_WRITE_ | (Offset & E2P_CMD_EPC_ADDR_MASK_);
		Status = Lan7800WriteRegister(Adapter, E2P_CMD, Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		Status = Lan7800WaitEeprom(Adapter);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Offset++;
	}

	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800GetEepromLength(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	OUT UINT32				*Length
	)
{
	UINT32		i;
	EFI_STATUS	Status = EFI_SUCCESS;
	UINT8		Save[4];
	UINT8		SaveEach[4];

	//Save first 4 bytes
	Status = Lan7800ReadEeprom(Adapter, 0, 4, Save); 
	if (EFI_ERROR(Status)) {
		*Length = 0;
		return Status;
	}

	//Read 4 bytes every 128 byte offset and compare with saved
	for (i = 128; i <= MAX_EEPROM_SIZE; i += 128) {
		Status = Lan7800ReadEeprom(Adapter, i, 4, SaveEach); 
		if (EFI_ERROR(Status)) {
			*Length = 0;
			return Status;
		}
		if (memcmp(Save, SaveEach, 4) == 0) {
			*Length = i;
			break;
		}
		memset(SaveEach, 0, sizeof(SaveEach));
	}
	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800ReadOtpInternal(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	)
{
	UINT32		i;
	UINT32		Value;		
	EFI_STATUS	Status;
	UINTN		Timeout = 0;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800ReadRegister(Adapter, OTP_PWR_DN, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	if (Value & OTP_PWR_DN_PWRDN_N_) {
		// clear it and wait to be cleared
		Status = Lan7800WriteRegister(Adapter, OTP_PWR_DN, 0);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		while (Timeout < 3) {
			Status = Lan7800ReadRegister(Adapter, OTP_PWR_DN, &Value);
			if (EFI_ERROR(Status)) {
				return Status;
			}
			if (!(Value & OTP_PWR_DN_PWRDN_N_)) {
				break;
			}
			gBS->Stall(100);
			Timeout++;
		}
	}

	for (i = 0; i < Length; i++) {
		Status = Lan7800WriteRegister(Adapter, OTP_ADDR1, ((Offset + i) >> 8) & OTP_ADDR1_15_11);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Status = Lan7800WriteRegister(Adapter, OTP_ADDR2, ((Offset + i) & OTP_ADDR2_10_3));
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Status = Lan7800WriteRegister(Adapter, OTP_FUNC_CMD, OTP_FUNC_CMD_READ_);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Status = Lan7800WriteRegister(Adapter, OTP_CMD_GO, OTP_CMD_GO_GO_);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		Timeout = 0;
		while (Timeout < 3) {
			Status = Lan7800ReadRegister(Adapter, OTP_STATUS, &Value);
			if (EFI_ERROR(Status)) {
				return Status;
			}
			if (!(Value & OTP_STATUS_BUSY_)) {
				break;
			}
			gBS->Stall(100);
			Timeout++;
		}
		Status = Lan7800ReadRegister(Adapter, OTP_RD_DATA, &Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Data[i] = (UINT8) (Value & 0xFF);
	}
	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800ReadOtp(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	)
{
	UINT8		Signature;
	EFI_STATUS	Status;

	Status = Lan7800ReadOtpInternal(Adapter, 0, 1, &Signature);
	if (Status == EFI_SUCCESS) {
		if (Signature == OTP_INDICATOR_2) {
			Offset += 0x100;
		}
		Status = Lan7800ReadOtpInternal(Adapter, Offset, Length, Data);
	}
	return Status;
}

BOOLEAN
IsZeroMacAddress(
	IN UINT8	*MacAddr
	)
{
	return !(MacAddr[0] | MacAddr[1] | MacAddr[2] | MacAddr[3] | MacAddr[4] | MacAddr[5]);
}

BOOLEAN
IsMulticaseMacAddress(
  IN UINT8	*MacAddr
  )
{
	return (0x01 & MacAddr[0]);
}

BOOLEAN
IsValidMacAddress(
	IN UINT8	*MacAddr
	)
{
	return !IsMulticaseMacAddress(MacAddr) && !IsZeroMacAddress(MacAddr);
          
}

BOOLEAN
IsBroadcastMacAddress(
	IN UINT8	*MacAddr
	)
{
	return ((MacAddr[0] & MacAddr[1] & MacAddr[2] & MacAddr[3] & MacAddr[4] & MacAddr[5]) == 0xff);
}

VOID 
GenerateRandomMacAddress(
	IN UINT8	*MacAddr
	)
{
	MacAddr[0] = 0x00;
	MacAddr[0] = 0x80;        /* clear multicast bit */
	MacAddr[0] = 0x0F;        /* set local assignment bit (IEEE802) */
	MacAddr[1] = (UINT8) (NetRandomInitSeed() & 0xFF);
	MacAddr[2] = (UINT8) (NetRandomInitSeed() & 0xFF);
	MacAddr[3] = (UINT8) (NetRandomInitSeed() & 0xFF);
	MacAddr[4] = (UINT8) (NetRandomInitSeed() & 0xFF);
	MacAddr[5] = (UINT8) (NetRandomInitSeed() & 0xFF);
}


EFI_STATUS 
Lan7800InitMacAddress(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		AddrLow, AddrHi;
	UINT8		MacAddress[6];
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	Status = Lan7800ReadRegister(Adapter, RX_ADDRL, &AddrLow);
	if (EFI_ERROR (Status)) {
		return Status;
	}
	Status = Lan7800ReadRegister(Adapter, RX_ADDRH, &AddrHi);
	if (EFI_ERROR (Status)) {
		return Status;
	}
	MacAddress[0] = AddrLow & 0xFF;
	MacAddress[1] = (AddrLow >> 8) & 0xFF;
	MacAddress[2] = (AddrLow >> 16) & 0xFF;
	MacAddress[3] = (AddrLow >> 24) & 0xFF;
	MacAddress[4] = AddrHi & 0xFF;
	MacAddress[5] = (AddrHi >> 8) & 0xFF;

	if (!IsValidMacAddress(MacAddress)) {
		/* reading mac address from EEPROM or OTP */
		Status = Lan7800ReadEeprom(Adapter, EEPROM_MAC_OFFSET, ETH_ALEN, MacAddress); 
		if (EFI_ERROR(Status)) {
			return Status;
		}
		//Check whether it is valid Mac address else read from OTP
		if (!IsValidMacAddress(MacAddress)) {
			Status = Lan7800ReadOtp(Adapter, EEPROM_MAC_OFFSET, ETH_ALEN, MacAddress);
			if (!IsValidMacAddress(MacAddress)) {
				//If no valid Mac address, generate one
				GenerateRandomMacAddress(MacAddress);
			}
		}

		AddrLow = MacAddress[0] | (MacAddress[1] << 8) | (MacAddress[2] << 16) | (MacAddress[3] << 24);
		AddrHi = MacAddress[4] | (MacAddress[5] << 8);

		Status = Lan7800WriteRegister(Adapter, RX_ADDRL, AddrLow);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		Status = Lan7800WriteRegister(Adapter, RX_ADDRH, AddrHi);
		if (EFI_ERROR(Status)) {
			return Status;
		}
	}
	Status = Lan7800WriteRegister(Adapter, MAF_LO(0), AddrLow);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WriteRegister(Adapter, MAF_HI(0), AddrHi | MAF_HI_VALID_);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Save the permanenet and current Mac address in the Adapter structure
	CopyMem(Adapter->PermanentMacAddress, MacAddress, PXE_HWADDR_LEN_ETHER);
	CopyMem(Adapter->MacAddress, MacAddress, PXE_HWADDR_LEN_ETHER);
	SetMem(Adapter->BroadcastMacAddress, PXE_HWADDR_LEN_ETHER, 0xFF);

	DEBUGPRINT(DBG_2021, ("MacAddress = %02x:%02x:%02x:%02x:%02x:%02x\n",
		MacAddress[0], MacAddress[1], MacAddress[2], MacAddress[3], MacAddress[4], MacAddress[5]));

	return Status;
}

EFI_STATUS
Lan7800PhyInitialize(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32	Value;
	UINT32 count = 0;
	EFI_STATUS   Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));
	DEBUGPRINT(DBG_2021, ("%a\n", __FUNCTION__));

	// Auto-neg advertisement
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_AUTONEG_ADV, &Value);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("%a Lan7800ReadPhyRegister ERROR\n", __FUNCTION__));
		return Status;
	}
	Value |= (NWAY_AR_ALL_CAPS_ | NWAY_AR_ASM_DIR_ | NWAY_AR_PAUSE_);
	Status = Lan7800WritePhyRegister(Adapter, Adapter->PhyId, PHY_AUTONEG_ADV, Value);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("%a Lan7800WritePhyRegister ERROR\n", __FUNCTION__));
		return Status;
	}

	// 1000BASE-T Control register setup
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_1000T_CTRL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= CR_1000T_FD_CAPS_;
	// MAC do not support 1000HD
	Value &= ~CR_1000T_HD_CAPS_;
	Status = Lan7800WritePhyRegister(Adapter, Adapter->PhyId, PHY_1000T_CTRL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	// Restart Auto-negotiation
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_CTRL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= (MII_CR_AUTO_NEG_EN_ | MII_CR_RESTART_AUTO_NEG_);
	Status = Lan7800WritePhyRegister(Adapter, Adapter->PhyId, PHY_CTRL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	
	// check autoneg done
	while (count <100) {
		Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_STATUS, &Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}
		if (Value & MII_SR_AUTONEG_COMPLETE_)
			break;
		count++;
		gBS->Stall(50000);	// (unit nS) 2/23/2021
	}
	if (count == 100)
		DEBUGPRINT(DBG_ERROR, ("%a timeout autoneg\n", __FUNCTION__));
			
	
	DEBUGPRINT(DBG_2021, ("%a success\n", __FUNCTION__));


	return Status;
}

EFI_STATUS
Lan7800StartTransmit(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32	Value;
	EFI_STATUS   Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	// Transmitter enable
	Status = Lan7800ReadRegister(Adapter, MAC_TX, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= MAC_TX_TXEN_;
	Status = Lan7800WriteRegister(Adapter, MAC_TX, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	// Enable transmitting frames to MAC
	Status = Lan7800ReadRegister(Adapter, FCT_TX_CTL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= FCT_TX_CTL_EN_;
	Status = Lan7800WriteRegister(Adapter, FCT_TX_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS
Lan7800StopTransmit(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32	Value;
	EFI_STATUS   Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	//FIXME: Check to see any pending packet, flush

	// Transmitter enable
	Status = Lan7800ReadRegister(Adapter, MAC_TX, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value &= ~MAC_TX_TXEN_;
	Status = Lan7800WriteRegister(Adapter, MAC_TX, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	// Enable transmitting frames to MAC
	Status = Lan7800ReadRegister(Adapter, FCT_TX_CTL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value &= ~FCT_TX_CTL_EN_;
	Status = Lan7800WriteRegister(Adapter, FCT_TX_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS
Lan7800StopReceive(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32	Value;
	EFI_STATUS   Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	//FIXME: Check to see any pending packet, flush

	// Receiver enable
	Status = Lan7800ReadRegister(Adapter, MAC_RX, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value &= ~MAC_RX_RXEN_;
	Status = Lan7800WriteRegister(Adapter, MAC_RX, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	// Enable to accept traffic from RFE to Rx FIFO
	Status = Lan7800ReadRegister(Adapter, FCT_RX_CTL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value &= ~FCT_RX_CTL_EN_;
	Status = Lan7800WriteRegister(Adapter, FCT_RX_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS
Lan7800StartReceive(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		Value;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));
	
	// Receiver enable
	Status = Lan7800ReadRegister(Adapter, MAC_RX, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= MAC_RX_RXEN_;
	Status = Lan7800WriteRegister(Adapter, MAC_RX, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	// Enable to accept traffic from RFE to Rx FIFO
	Status = Lan7800ReadRegister(Adapter, FCT_RX_CTL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= FCT_RX_CTL_EN_;
	Status = Lan7800WriteRegister(Adapter, FCT_RX_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS 
Lan7800Reset(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		Value;
	UINT32		ChipId;
	UINT32		Count = 0;
	UINT32		Flow = 0, Fct_flow = 0;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));
	DEBUGPRINT(DBG_2021, ("%a\n", __FUNCTION__));

	//Do lite reset
	Status = Lan7800ReadRegister(Adapter, HW_CFG, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= HW_CFG_LRST_;
	Status = Lan7800WriteRegister(Adapter, HW_CFG, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	while (Count < 100) {
		gBS->Stall(10000);
		Status = Lan7800ReadRegister(Adapter, HW_CFG, &Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		if (!(Value & HW_CFG_LRST_)) {
			break;
		}
		Count++;
	}

	if (Count >= 100) {
		DEBUGPRINT(DBG_ERROR, ("timeout on completion of LiteReset\n"));
		return EFI_TIMEOUT;
	}

	//Do phy reset
	Status = Lan7800ReadRegister(Adapter, PMT_CTL, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= PMT_CTL_PHY_RST_;
	Status = Lan7800WriteRegister(Adapter, PMT_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	while (Count < 100) {
		gBS->Stall(10000);
		Status = Lan7800ReadRegister(Adapter, PMT_CTL, &Value);
		if (EFI_ERROR(Status)) {
			return Status;
		}

		if (!(Value & PMT_CTL_PHY_RST_)) {
			break;
		}
		Count++;
	}

	if (Count >= 100) {
		DEBUGPRINT(DBG_ERROR, ("timeout waiting for PHY Reset\n"));
		return EFI_TIMEOUT;
	}

	//Check if external phy
	Status = Lan7800ReadRegister(Adapter, ID_REV, &ChipId);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	if ((((ChipId & ID_REV_CHIP_ID_MASK_) >> 16)== 0x7801) || (((ChipId & ID_REV_CHIP_ID_MASK_) >> 16)== 0x780A)) {
		Adapter->ExternalPhy = 1;
	} else {
		Adapter->ExternalPhy = 0;
	}
	if (Adapter->ExternalPhy == 1) {
		for(Count =31; Count >= 0; Count--)
		{
			Adapter->PhyId = (UINT8) Count;
			Status = Lan7800ReadPhyRegister(Adapter, (UINT8) Count, PHY_STATUS, &Value);
			if (EFI_ERROR(Status)) {
				return Status;
			}
			if(((Value & 0xffff) != 0xffff) && ((Value & 0xffff) != 0x0000))
			{
				break;
			}
		}
		// if we did not find it default back to Id = 1.
		if (Count < 0)
		{
			Adapter->PhyId = (UINT8) 0x1;
		}
	}

	//Initialize Mac address
	Lan7800InitMacAddress(Adapter);

	// Aug 2021
	//Respond to IN token with ZLP in USB2, NRDY in USB3, note that NRDY only in lan78xx USB3 even if this bit is set to ZLP
	Status = Lan7800ReadRegister(Adapter, USB_CFG0, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value &= ~USB_CFG_BIR_;
	Status = Lan7800WriteRegister(Adapter, USB_CFG0, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Set the burst cap
	if (Adapter->DeviceSpeedCapbility >= 0x300) {
		Value =  DEFAULT_BURST_CAP_SIZE / SS_USB_PKT_SIZE;
	} else if (Adapter->DeviceSpeedCapbility == 0x210) {
		Value =  DEFAULT_BURST_CAP_SIZE / HS_USB_PKT_SIZE;
	} else {
		Value =  DEFAULT_BURST_CAP_SIZE / FS_USB_PKT_SIZE;
	}
	Status = Lan7800WriteRegister(Adapter, BURST_CAP, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Set Bulkin delay
	Status = Lan7800WriteRegister(Adapter, BULK_IN_DLY, 1); // set to 1 for better performance
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Enable MEF
	Status = Lan7800ReadRegister(Adapter, HW_CFG, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= HW_CFG_MEF_;
	Status = Lan7800WriteRegister(Adapter, HW_CFG, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Enable Burst cap
	Status = Lan7800ReadRegister(Adapter, USB_CFG0, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= USB_CFG_BCE_;
	Status = Lan7800WriteRegister(Adapter, USB_CFG0, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//set FIFO sizes
	Value = (MAX_RX_FIFO_SIZE - 512) / 512;
	Status = Lan7800WriteRegister(Adapter, FCT_RX_FIFO_END, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value = (MAX_TX_FIFO_SIZE - 512) / 512;
	Status = Lan7800WriteRegister(Adapter, FCT_TX_FIFO_END, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Clear interrupt status
	Status = Lan7800WriteRegister(Adapter, INT_STS, INT_STS_CLEAR_ALL_);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Disable FLOW control, enable it after link establishes
	Status = Lan7800WriteRegister(Adapter, FLOW, 0);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WriteRegister(Adapter, FCT_FLOW, 0);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//Set RFE_CTL, multicast will be enable later when set multicast
	Value = (RFE_CTL_BCAST_EN_ | RFE_CTL_DA_PERFECT_);
	Status = Lan7800WriteRegister(Adapter, RFE_CTL, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Initialize phy
	Status = Lan7800PhyInitialize(Adapter);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("Phy initialization failed %r\n", Status));
		return Status;
	}
	//Set GMII, auto duplex and speed. disable EEE
	Status = Lan7800ReadRegister(Adapter, MAC_CR, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= MAC_CR_GMII_EN_;
	Value |= (MAC_CR_AUTO_DUPLEX_ | MAC_CR_AUTO_SPEED_);
	Value &= ~MAC_CR_EEE_EN_;
	Status = Lan7800WriteRegister(Adapter, MAC_CR, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Enable flow control 7/21/2021
	Flow |= (FLOW_CR_TX_FCEN_ | 0xFFFF);
	Flow |= FLOW_CR_RX_FCEN_;
	if (Adapter->DeviceSpeedCapbility >= 0x300) {
		Fct_flow = FLOW_CTRL_THRESHOLD(FLOW_ON_SS, FLOW_OFF_SS);
	} else {
		Fct_flow = FLOW_CTRL_THRESHOLD(FLOW_ON_HS, FLOW_OFF_HS);
	}

	/* threshold value should be set before enabling flow */
	Status = Lan7800WriteRegister(Adapter, FCT_FLOW, Fct_flow);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WriteRegister(Adapter, FLOW, Flow);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//Enabling FCS stripping for received packets
	Status = Lan7800ReadRegister(Adapter, MAC_RX, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Value |= MAC_RX_FCS_STRIP_;
	Status = Lan7800WriteRegister(Adapter, MAC_RX, Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	
	DEBUGPRINT(DBG_2021, ("%a done\n", __FUNCTION__));

	return Status;
}

EFI_STATUS 
Lan7800DeviceInitialize(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	EFI_STATUS Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	//Initialize device related data
	Adapter->TxBufferSize = MAX_TX_FIFO_SIZE;
	Adapter->RxBufferSize = MAX_TX_FIFO_SIZE;

	//Internal phy id
	Adapter->PhyId = 1;

	Adapter->RxTxPathEnabled = 0;

	//Reset the device and initialize
	Status = Lan7800Reset(Adapter);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS 
Lan7800Transmit(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				*FrameAddr,
	IN UINT16				MeadiaHeaderLen,
	IN UINT32				DataLen
	)
{
	UINT32 TxCommandA;
	UINT32 TxCommandB;
	UINT32 PacketLen;
	UINTN  UrbLen;
	UINT32 UsbStatus;
	EFI_STATUS Status = EFI_SUCCESS;
	UINT8 *BuffPtr = Adapter->TxBuffer;

	//DEBUGPRINT(DBG_LAN,("%a\n", __FUNCTION__));

	PacketLen = MeadiaHeaderLen + DataLen;
	TxCommandA = (UINT32) (PacketLen & TX_CMD_A_LEN_MASK_) | TX_CMD_A_FCS_;
	TxCommandB = 0;

	CopyMem(BuffPtr, &TxCommandA, sizeof(TxCommandA));
	BuffPtr += sizeof(TxCommandA);
	CopyMem(BuffPtr, &TxCommandB, sizeof(TxCommandB));
	BuffPtr += sizeof(TxCommandB);
	CopyMem(BuffPtr, FrameAddr, PacketLen);
	UrbLen = PacketLen + 8;


    Status = Adapter->UsbIo->UsbBulkTransfer(
				Adapter->UsbIo,
				Adapter->UsbEndpointInfo.EndpointBulkOut,
				Adapter->TxBuffer,
				&UrbLen,
				10,		// 2021 use the smaller value for possible poformance improvement in mS
				&UsbStatus
				);

	//Store the buffer address, need to indicate this buffer in getstatus for reuse
	Adapter->TxUndiBuffer = FrameAddr;
	Adapter->UndiDeviceInterruptStatus |= PXE_STATFLAGS_GET_STATUS_TRANSMIT;
	return Status;
  
}

EFI_STATUS 
Lan7800Receive(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	EFI_STATUS	Status = EFI_SUCCESS;
	UINT8		*Buffer;
	UINTN		BuffLen;
	UINT32		UsbStatus = 0;

	Buffer = Adapter->RxBuffer;
	BuffLen = Adapter->RxBufferSize;

	DEBUGPRINT(DBG_LAN,("%a\n", __FUNCTION__));

	Adapter->RxBufferDataStart = Adapter->RxBuffer;

    Status = Adapter->UsbIo->UsbBulkTransfer(
                          Adapter->UsbIo,
                          Adapter->UsbEndpointInfo.EndpointBulkIn,
                          Buffer,
                          &BuffLen,
                          1, // This is the timeout how long this synchronous request should wait for the data. 8/23/2021
                          &UsbStatus
                          );
	if (EFI_ERROR(Status) || (UsbStatus != EFI_USB_NOERROR)) {
		return Status;
	}

	if (BuffLen) {
		//Mark data present in the Adapter Rx buffer
		Adapter->RxBufferDataLen = BuffLen;
		Adapter->RxBufferDataPresent = 1;
		Adapter->UndiDeviceInterruptStatus |= PXE_STATFLAGS_GET_STATUS_RECEIVE;
	} else {
			Adapter->RxBufferDataLen = 0;
			Adapter->RxBufferDataPresent = 0;
	}

	return Status;
}

UINT32
EtherCrc(
	UINTN	Length,
	UINT8	*Data
  )
{
	INT32   Crc = -1;
	UINT8   CurrentOctet;
	UINT8   Bit;
	STATIC CONST UINT32 mEthernetPolynomial = 0x04c11db7U;

	while (Length > 0) {
		CurrentOctet = *Data++;
		for (Bit = 0; Bit < 8; Bit++, CurrentOctet >>= 1) {
			Crc = (Crc << 1) ^ ((Crc < 0) ^ (CurrentOctet & 1) ? mEthernetPolynomial : 0);
		}
		--Length;
	}

	return (UINT32) Crc;
}
UINT32
Lan7800Hash(
	IN UINT8	*Addr
  )
{
  return (EtherCrc (6, Addr) >> 23) & 0x1FF;
}

EFI_STATUS
Lan7800WaitDataportNotBusy(
	IN LAN7800_ADAPTER_DATA	*Adapter
  )
{
	EFI_STATUS	Status;
	UINT32		Value;
	UINTN		Index;

	for (Index = 0; Index < 100; Index++) {
		Status = Lan7800ReadRegister(Adapter, DP_SEL, &Value);
		if (Value & DP_SEL_DPRDY_) {
			return EFI_SUCCESS;
		}

		gBS->Stall (40);
	}
	return EFI_TIMEOUT;
}

EFI_STATUS
Lan7800WriteDataport(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT32				RamSelect,
	IN UINT32				Address,
	IN UINT32				Length,
	IN UINT32				*Buffer
  )
{
	EFI_STATUS	Status;
	UINT32		Value;
	UINTN		Index;

	Status = Lan7800WaitDataportNotBusy(Adapter);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	Status = Lan7800ReadRegister(Adapter, DP_SEL, &Value);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	Value &= ~DP_SEL_RSEL_MASK_;
	Value |= RamSelect;
	Status = Lan7800WriteRegister(Adapter, DP_SEL, Value);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	for (Index = 0; Index < Length; ++Index) {
		Status = Lan7800WriteRegister(Adapter, DP_ADDR, (UINT32)(Address + Index));
		if (EFI_ERROR (Status)) {
			return Status;
		}

		Status = Lan7800WriteRegister(Adapter, DP_DATA, Buffer[Index]);
		if (EFI_ERROR (Status)) {
			return Status;
		}

		Status = Lan7800WriteRegister(Adapter, DP_CMD, DP_CMD_WRITE_);
		if (EFI_ERROR (Status)) {
			return Status;
		}
    
		Status = Lan7800WaitDataportNotBusy(Adapter);
		if (EFI_ERROR (Status)) {
			return Status;
		}
	}

	return EFI_SUCCESS;
}
EFI_STATUS 
Lan7800SetMulticast(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32 BitNum;
	UINTN  Index;
	UINT32 RfeCtl = 0;
	EFI_STATUS	Status = EFI_SUCCESS;
	UINT32 McastHashTable[DP_SEL_VHF_HASH_LEN];

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	ZeroMem(McastHashTable, sizeof (McastHashTable));

	Status = Lan7800ReadRegister(Adapter, RFE_CTL, &RfeCtl);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	//Reset multicast filter
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST) {
		RfeCtl &= ~RFE_CTL_MCAST_HASH_;
	}

	//enable/disable promiscuos mode
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
			RfeCtl |= (RFE_CTL_MCAST_EN_ | RFE_CTL_UCAST_EN_);
	}else{
			RfeCtl &= ~(RFE_CTL_MCAST_EN_ | RFE_CTL_UCAST_EN_);
	}

	//all multicast
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
			RfeCtl |= RFE_CTL_MCAST_EN_;
	}else{
			RfeCtl &= ~(RFE_CTL_MCAST_EN_);
	}
	//all unicast
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_UNICAST) {
			RfeCtl |= RFE_CTL_UCAST_EN_;
	}else{
			RfeCtl &= ~(RFE_CTL_UCAST_EN_);
	}
	//broadcast
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
			RfeCtl |= RFE_CTL_BCAST_EN_;
	}else{
			RfeCtl &= ~(RFE_CTL_BCAST_EN_);
	}

	//Receive multicast filter
	if (Adapter->ReceiveFilterFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) {
			RfeCtl |= RFE_CTL_MCAST_HASH_;
			//Calculate the hash value
			for (Index = 0; Index < Adapter->MulticastListCount; Index++) {
				BitNum = Lan7800Hash((UINT8*)&Adapter->MultiCastList[Index]);
				McastHashTable[BitNum / 32] |= (0x01 << (BitNum % 32));
			}
	}else{
			RfeCtl &= ~RFE_CTL_MCAST_HASH_;
	}
	//Set the multicast filter
	if (RfeCtl & RFE_CTL_MCAST_HASH_) {
		Status = Lan7800WriteDataport(Adapter, DP_SEL_RSEL_VLAN_DA_, DP_SEL_VHF_VLAN_LEN,
					DP_SEL_VHF_HASH_LEN, McastHashTable);
		if (EFI_ERROR (Status)) {
			return Status;
		}
	}
	
	// Set the RFE_CTL register here 5/21/2021	
	Status = Lan7800WriteRegister(Adapter, RFE_CTL, RfeCtl);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	if (!Adapter->RxTxPathEnabled) {
		Lan7800StartTransmit(Adapter);
		Lan7800StartReceive(Adapter);
		Adapter->RxTxPathEnabled = 1;
		
		DEBUGPRINT(DBG_2021, ("%a, RxTxPathEnabled\n", __FUNCTION__));

	}

	return EFI_SUCCESS;
}

EFI_STATUS 
Lan7800SetMacAddress(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	UINT32		AddrLow, AddrHi;
	EFI_STATUS	Status = EFI_SUCCESS;

	DEBUGPRINT(DBG_LAN, ("%a\n", __FUNCTION__));

	AddrLow = Adapter->MacAddress[0] | (Adapter->MacAddress[1] << 8) | 
				(Adapter->MacAddress[2] << 16) | (Adapter->MacAddress[3] << 24);
	AddrHi = Adapter->MacAddress[4] | (Adapter->MacAddress[5] << 8);

	Status = Lan7800WriteRegister(Adapter, RX_ADDRL, AddrLow);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WriteRegister(Adapter, RX_ADDRH, AddrHi);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	Status = Lan7800WriteRegister(Adapter, MAF_LO(0), AddrLow);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800WriteRegister(Adapter, MAF_HI(0), AddrHi | MAF_HI_VALID_);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	return Status;
}

EFI_STATUS 
Lan7800LinkCheck(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	UINT16					*Flags
	)
{
	UINT32		Value;
	EFI_STATUS	Status = EFI_SUCCESS;

	//read twice 
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_STATUS, &Value);
	if (EFI_ERROR(Status)) {
		DEBUGPRINT(DBG_ERROR, ("%a Lan7800ReadPhyRegister ERROR\n", __FUNCTION__));
		return Status;
	}
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_STATUS, &Value);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	if (Value & MII_SR_LINK_STATUS_) {
		*Flags = 1;
	} else {
		*Flags = 0;
	}
	return EFI_SUCCESS;
}

VOID
PacketDump(
	UINT8	*Data,
	UINT32	Length
	)
{
	UINT32 Index;

	for (Index = 0; Index <= Length; Index++) {
		DEBUGPRINT(DBG_WARNING, ("%02x ", *(Data + Index)));		
	}
	DEBUGPRINT(DBG_WARNING, ("\n"));	
}

EFI_STATUS
Lan7800GetLinkStatus(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	OUT	UINT32				*LinkSpeed,
	OUT	UINT32				*Duplex
	)
{
	UINT32	Bmcr;
	UINT32	Bmsr;
	UINT32	Anar;
	UINT32	Anlpar;
	UINT32	Common;
	UINT32	ctrl1000;
	UINT32	stat1000;
	EFI_STATUS	Status = EFI_SUCCESS;

	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_CTRL, &Bmcr);
	if (EFI_ERROR(Status)) {
		return Status;
	}	
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_STATUS, &Bmsr);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	//read twice (link status bit latch low)
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_STATUS, &Bmsr);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_AUTONEG_ADV, &Anar);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_LP_ABILITY, &Anlpar);
	if (EFI_ERROR(Status)) {
		return Status;
	}
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_1000T_CTRL, &ctrl1000);
	if (EFI_ERROR(Status)) {
		return Status;
	}	
	Status = Lan7800ReadPhyRegister(Adapter, Adapter->PhyId, PHY_1000T_STATUS, &stat1000);
	if (EFI_ERROR(Status)) {
		return Status;
	}

	//check link status
	if (!(Bmsr & MII_SR_LINK_STATUS_)) {
		*LinkSpeed = SPEED_0;
		*Duplex = 0;	
		return Status;
	}

	Common = Anar & Anlpar;
	if ((ctrl1000 & CR_1000T_FD_CAPS_) && (stat1000 & SR_1000T_LP_FD_CAPS_)) {
		*LinkSpeed = SPEED_1000;
		*Duplex = FULL_DUPLEX;	
	} else {
		if (Common & NWAY_AR_100TX_FD_CAPS_) {
			*LinkSpeed = SPEED_100;
			*Duplex = FULL_DUPLEX;
		} else if (Common & NWAY_AR_100TX_HD_CAPS_) {
			*LinkSpeed = SPEED_100;
			*Duplex = HALF_DUPLEX;
		} else if (Common & NWAY_AR_10T_FD_CAPS_) {
			*LinkSpeed = SPEED_10;
			*Duplex = FULL_DUPLEX;
		} else if (Common & NWAY_AR_10T_HD_CAPS_) {
			*LinkSpeed = SPEED_10;
			*Duplex = HALF_DUPLEX;
		}
	}

	return Status;
}

EFI_STATUS
Lan7800StandardDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800ExtendedDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	return EFI_SUCCESS;
}

EFI_STATUS
Lan7800ManufacturingDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	)
{
	return EFI_SUCCESS;
}




