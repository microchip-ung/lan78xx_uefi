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

#include "Lan7800Utility.h"

EFI_INPUT_KEY    mKey;
UINTN            mMaxRow;

CONFIG_FUNCTION  mConfigFuncList[] = {
  {
    ViewEditEeprom,
    L"View/Edit EEPROM"
  },
  {
    ViewProperty,
    L"View Property"
  },
  {
    ViewEditRegister,
    L"Read/Write Register"
  },
  {
    ViewEditPhyRegister,
    L"Read/Write Phy Register"
  },
  {
    DumpOtp,
    L"Dump Otp Contents"
  },
  {
    NULL,
    NULL
  },
};


EFI_STATUS
WaitForSingleEvent(
	IN EFI_EVENT        Event,
	IN UINT64           Timeout OPTIONAL
	)
/*
Routine Description:
  Function waits for a given event to fire, or for an optional timeout to expire.
Arguments:
  Event            - The event to wait for
  Timeout          - An optional timeout value in 100 ns units.
Returns:
  EFI_SUCCESS       - Event fired before Timeout expired.
  EFI_TIME_OUT     - Timout expired before Event fired..
*/
{
	EFI_STATUS	Status;
	UINTN		Index;
	EFI_EVENT	TimerEvent;
	EFI_EVENT	WaitList[2];

	if (Timeout) {
		// Create a timer event
		Status = gBS->CreateEvent(EVT_TIMER, 0, NULL, NULL, &TimerEvent);
		if (!EFI_ERROR(Status)) {
			// Set the timer event
			gBS->SetTimer(TimerEvent, TimerRelative, Timeout);

			// Wait for the original event or the timer
			WaitList[0] = Event;
			WaitList[1] = TimerEvent;
			Status = gBS->WaitForEvent(2, WaitList, &Index);
			gBS->CloseEvent(TimerEvent);

			// If the timer expired, change the return to timed out
			if (!EFI_ERROR(Status)  &&  Index == 1) {
				Status = EFI_TIMEOUT;
			}
		}
	} else {
		// No timeout... just wait on the event
		Status = gBS->WaitForEvent(1, &Event, &Index);
	}

	return Status;
}


EFI_STATUS
WaitForKeyEvent(
	IN EFI_EVENT	Event,
	IN UINT64		Timeout OPTIONAL
	)
/*
Routine Description:
  Function waits for a given event to fire, or for an optional timeout to expire.
Arguments:
  Event             - The event to wait for
  Timeout           - An optional timeout value in 100 ns units.
Returns:
  EFI_SUCCESS       - Event fired before Timeout expired.
  EFI_TIME_OUT      - Timout expired before Event fired..
*/
{
	EFI_STATUS  Status;
	UINTN       Index;
	EFI_EVENT   TimerEvent;
	EFI_EVENT   WaitList[2];

	if (Timeout) {
		// Create a timer event
		Status = gBS->CreateEvent(EVT_TIMER, 0, NULL, NULL, &TimerEvent);
		if (!EFI_ERROR(Status)) {

			// Set the timer event
			gBS->SetTimer(TimerEvent, TimerRelative, Timeout);

			// Wait for the original event or the timer
			WaitList[0] = Event;
			WaitList[1] = TimerEvent;
			Status = gBS->WaitForEvent(2, WaitList, &Index);
			gBS->CloseEvent(TimerEvent);

			// If the timer expired, change the return to timed out
			if (!EFI_ERROR(Status) && (Index == 1)) {
				Status = EFI_TIMEOUT;
			}
		}
	} else {
		// No timeout... just wait on the event
		Status = gBS->WaitForEvent(1, &Event, &Index);
	}
	return Status;
}


VOID
ConMoveCursorBackward(
	IN UINTN		LineLength,
	IN OUT UINTN	*Column,
	IN OUT UINTN	*Row
  )
/*
Routine Description:
  Move the cursor position one character backward.
Arguments:
  LineLength       Length of a line. Get it by calling QueryMode
  Column           Current column of the cursor position
  Row              Current row of the cursor position
Returns:
*/
{
	// If current column is 0, move to the last column of the previous line,
	// otherwise, just decrement column.
	if (*Column == 0) {
		(*Column) = LineLength - 1;
		if (*Row > 0) {
			(*Row) --;
		}
	} else {
		(*Column) --;
	}
}

VOID
ConMoveCursorForward(
	IN UINTN		LineLength,
	IN UINTN		TotalRow,
	IN OUT UINTN	*Column,
	IN OUT UINTN	*Row
	)
/*
Routine Description:
  Move the cursor position one character backward.
Arguments:
  LineLength       Length of a line. Get it by calling QueryMode
  TotalRow         Total row of a screen, get by calling QueryMode
  Column           Current column of the cursor position
  Row              Current row of the cursor position
Returns:
*/
{
	// If current column is at line end, move to the first column of the nest
	// line, otherwise, just increment column.
	(*Column) ++;
	if (*Column >= LineLength) {
		(*Column) = 0;
		if ( (*Row) < TotalRow - 1) {
			(*Row) ++;
		}
	}
}

VOID
Input(
	IN CHAR16			*Prompt OPTIONAL,
	OUT CHAR16			*InStr,
	IN UINTN			StrLength,
	IN BOOLEAN			(*Filter) (CHAR16),
	IN BOOLEAN			*Enter,
	OUT EFI_INPUT_KEY	*RetKey
	)
/*
Routine Description:
  Input a string at the current cursor location, for StrLength
Arguments:
  Prompt          Prompt string
  InStr           Buffer to hold the input string
  StrLength       Length of the buffer
  Filter          You allow the key
  RetKey          read a key, if necessary
Returns:
  void
*/
{
	BOOLEAN							Done;
	UINTN							Column;
	UINTN							Row;
	UINTN							StartColumn;
	UINTN							Update;
	UINTN							Delete;
	UINTN							Len;
	UINTN							StrPos;
	UINTN							Index;
	UINTN							LineLength;
	UINTN							TotalRow;
	UINTN							SkipLength;
	UINTN							OutputLength;
	UINTN							TailRow;
	UINTN							TailColumn;
	BOOLEAN							InsertMode;
	EFI_INPUT_KEY					Key;
	EFI_SIMPLE_TEXT_OUTPUT_PROTOCOL	*ConOut = gST->ConOut;
	EFI_SIMPLE_TEXT_INPUT_PROTOCOL	*ConIn = gST->ConIn;

	if (Prompt) {
		ConOut->OutputString(ConOut, Prompt);
	}

	// Read a line from the console
	Len = StrLen(InStr);
	if (Len == 0) {
		StrPos = 0;
		OutputLength = 0;
		Update = 0;
		Delete = 0;
		InsertMode = TRUE;
	} else {
		StrPos = 0;
		OutputLength = 1;
		Update = (UINTN) - 1;
		Delete = 0;
		InsertMode = FALSE;
	}

	// If buffer is not large enough to hold a CHAR16, do nothing.
	if (StrLength < 1) {
		return;
	}

	// Get the screen setting and the current cursor location
	StartColumn = ConOut->Mode->CursorColumn;
	Column = StartColumn;
	Row = ConOut->Mode->CursorRow;
	ConOut->QueryMode(ConOut, ConOut->Mode->Mode, &LineLength, &TotalRow);
	if (LineLength == 0) {
		return;
	}

	if (Len == 0) {
		ZeroMem(InStr, StrLength * sizeof (CHAR16));
	} else {
		UINTN CurrX = ConOut->Mode->CursorColumn;
		UINTN CurrY = ConOut->Mode->CursorRow;
		Print(InStr);
		ConOut->SetCursorPosition(ConOut, CurrX, CurrY);
	}

	Done = FALSE;
	do {
		// Read a key
		WaitForSingleEvent(ConIn->WaitForKey, 0);
		ConIn->ReadKeyStroke(ConIn, &Key);

		switch (Key.UnicodeChar) {
		case CHAR_CARRIAGE_RETURN:
			// All done, print a newline at the end of the string
			TailRow = Row + (Len - StrPos + Column) / LineLength;
			TailColumn = (Len - StrPos + Column) % LineLength;
			Done = TRUE;
			if (Enter != NULL){
				*Enter = TRUE;
			}
			break;

		case CHAR_BACKSPACE:
			if (StrPos) {
				// If not move back beyond string beginning, move all characters behind
				// the current position one character forward
				StrPos -= 1;
				Update = StrPos;
				Delete = 1;
				CopyMem(InStr + StrPos, InStr + StrPos + 1, sizeof (CHAR16) * (Len - StrPos));
        
				// Adjust the current column and row
				ConMoveCursorBackward(LineLength, &Column, &Row);
			}
			break;

		default:
			if (Filter(Key.UnicodeChar)) {
				// If we are at the buffer's end, drop the key
				if (Len == StrLength - 1 && (InsertMode || StrPos == Len)) {
					break;
				}

				// If in insert mode, move all characters behind the current position
				// one character backward to make space for this character. Then store
				// the character.
				if (InsertMode) {
					for (Index = Len; Index > StrPos; Index -= 1) {
						InStr[Index] = InStr[Index - 1];
					}
				}
				InStr[StrPos] = Key.UnicodeChar;
				Update = StrPos;
				StrPos += 1;
				OutputLength = 1;
			}
			break;

		case CHAR_NULL:
			switch (Key.ScanCode) {
			case SCAN_DELETE:
				// Move characters behind current position one character forward
				if (Len) {
					Update = StrPos;
					Delete = 1;
					CopyMem(InStr + StrPos, InStr + StrPos + 1, sizeof (CHAR16) * (Len - StrPos));
				}
				break;

			case SCAN_LEFT:
				// Adjust current cursor position
				if (StrPos) {
					StrPos -= 1;
					ConMoveCursorBackward(LineLength, &Column, &Row);
				}
				break;

			case SCAN_RIGHT:
				// Adjust current cursor position
				if (StrPos < Len) {
					StrPos += 1;
					ConMoveCursorForward(LineLength, TotalRow, &Column, &Row);
				}
			break;

			case SCAN_HOME:
				// Move current cursor position to the beginning of the command line
				Row -= (StrPos + StartColumn) / LineLength;
				Column = StartColumn;
				StrPos = 0;
				break;

			case SCAN_END:     
				// Move current cursor position to the end of the command line
				TailRow = Row + (Len - StrPos + Column) / LineLength;
				TailColumn = (Len - StrPos + Column) % LineLength;
				Row = TailRow;
				Column = TailColumn;
				StrPos = Len;
				break;

			case SCAN_ESC:
				// Prepare to clear the current command line
				InStr[0] = 0;
				Update = 0;
				Delete = Len;
				Row -= (StrPos + StartColumn) / LineLength;
				Column = StartColumn;
				OutputLength = 0;
				if (RetKey != NULL) {
					Done = TRUE;
				}
				if(Enter != NULL) {
					*Enter = FALSE;
					Done = TRUE;
				}
				break;

			case SCAN_INSERT:
				// Toggle the SEnvInsertMode flag
				InsertMode = (BOOLEAN) !InsertMode;
				break;
			}
		}

		if (Done) {
			break;
		}

		// If we need to update the output do so now
		if (Update != -1) {
			ConOut->SetCursorPosition(ConOut, Column, Row);
			Print (L"%s%*s", InStr + Update, Delete, L"");
			Len = StrLen (InStr);

			if (Delete) {
				ZeroMem (InStr + Len, Delete * sizeof (CHAR16));
			}

			if (StrPos > Len) {
				StrPos = Len;
			}

			Update = (UINTN) (-1);

			// After using print to reflect newly updates, if we're not using
			// BACKSPACE and DELETE, we need to move the cursor position forward,
			// so adjust row and column here.
			if (Key.UnicodeChar != CHAR_BACKSPACE && !(Key.UnicodeChar == 0 && Key.ScanCode == SCAN_DELETE)) {
				// Calulate row and column of the tail of current string
				TailRow = Row + (Len - StrPos + Column + OutputLength) / LineLength;
				TailColumn = (Len - StrPos + Column + OutputLength) % LineLength;

				// If the tail of string reaches screen end, screen rolls up, so if
				// Row does not equal TailRow, Row should be decremented
				
				// (if we are recalling commands using UPPER and DOWN key, and if the
				// old command is too long to fit the screen, TailColumn must be 79.

				if (TailColumn == 0 && TailRow >= TotalRow && Row != TailRow) {
					--Row;
				}

				// Calculate the cursor position after current operation. If cursor
				// reaches line end, update both row and column, otherwise, only
				// column will be changed.
				if (Column + OutputLength >= LineLength) {
					SkipLength = OutputLength - (LineLength - Column);

					Row += SkipLength / LineLength + 1;
					if (Row > TotalRow - 1) {
						Row = TotalRow - 1;
					}
					Column = SkipLength % LineLength;
				} else {
					Column += OutputLength;
				}
			}
			Delete = 0;
		}

		// Set the cursor position for this key
		ConOut->SetCursorPosition(ConOut, Column, Row);
	} while (!Done);

	// Return the data to the caller
	if (RetKey != NULL) {
		RetKey->ScanCode = Key.ScanCode;
		RetKey->UnicodeChar = Key.UnicodeChar;
	}
  return;
}

BOOLEAN
IsYesNoChar(
  IN CHAR16	Character
  )
{
  return (BOOLEAN) ( (L'Y' == Character) || (Character == L'N') ||
                     (L'y' == Character) || (Character == L'n'));
}

BOOLEAN
IsHexDigitChar(
  IN  CHAR16    Character
  )
{
	return (BOOLEAN) ( (L'0' <= Character) && (Character <= L'9') ||
			(L'a' <= Character) && (Character <= L'f') ||
			(L'A' <= Character) && (Character <= L'F'));
}

BOOLEAN
IsDigitChar(
	IN CHAR16	Character
  )
{
	return (BOOLEAN) ( (L'0' <= Character) && (Character <= L'9'));
}

BOOLEAN
IsAlphaChar(
	IN CHAR16	Character
  )
{
	return (BOOLEAN) ( (L'a' <= Character) && (Character <= L'z') ||
			(L'A' <= Character) && (Character <= L'Z'));
}

BOOLEAN
IsAlNumChar(
	IN CHAR16	Character
	)
{
	return (BOOLEAN) ( (L'0' <= Character) && (Character <= L'9') ||
			(L'a' <= Character) && (Character <= L'z') ||
			(L'A' <= Character) && (Character <= L'Z'));
}

VOID
MessagePopUp(
	IN UINTN	Attribute,                
	IN CHAR16	*Message
	)
{
	EFI_SIMPLE_TEXT_OUTPUT_PROTOCOL	*ConOut;
	EFI_SIMPLE_TEXT_OUTPUT_MODE		SavedConsoleMode;
	UINTN							Columns;
	UINTN							Rows;
	UINTN							Column;
	UINTN							Row;
	UINTN							NumberOfLines;
	UINTN							MaxLength;
	UINTN							Length;
	CHAR16							*Line;

	// Do not accept NULL message
	if (Message == NULL) {
		return;
	}
	MaxLength = StrLen(Message);
	NumberOfLines = 1;

	// Cache a pointer to the Simple Text Output Protocol in the EFI System Table
	ConOut = gST->ConOut;
  
	// Save the current console cursor position and attributes
	CopyMem(&SavedConsoleMode, ConOut->Mode, sizeof (SavedConsoleMode));

	// Retrieve the number of columns and rows in the current console mode
	ConOut->QueryMode(ConOut, SavedConsoleMode.Mode, &Columns, &Rows);

	// Disable cursor and set the foreground and background colors specified by Attribute
	ConOut->EnableCursor(ConOut, FALSE);
	ConOut->SetAttribute(ConOut, Attribute);

	// Limit NumberOfLines to height of the screen minus 3 rows for the box itself
	NumberOfLines = MIN(NumberOfLines, Rows - 3);

	// Limit MaxLength to width of the screen minus 2 columns for the box itself
	MaxLength = MIN(MaxLength, Columns - 2);

	// Compute the starting row and starting column for the popup
	Row    = (Rows - (NumberOfLines + 3)) / 2;
	Column = (Columns - (MaxLength + 2)) / 2;

	// Allocate a buffer for a single line of the popup with borders and a Null-terminator
	Line = AllocateZeroPool ((MaxLength + 3) * sizeof (CHAR16));

	// Draw top of popup box   
	SetMem16 (Line, (MaxLength + 2) * 2, BOXDRAW_HORIZONTAL);
	Line[0]             = BOXDRAW_DOWN_RIGHT;
	Line[MaxLength + 1] = BOXDRAW_DOWN_LEFT;
	Line[MaxLength + 2] = L'\0';
	ConOut->SetCursorPosition(ConOut, Column, Row++);
	ConOut->OutputString(ConOut, Line);

	// Draw middle of the popup with strings
	while (NumberOfLines > 0) {
		Length = StrLen (Message);
		SetMem16 (Line, (MaxLength + 2) * 2, L' ');
		if (Length <= MaxLength) {
			// Length <= MaxLength
			CopyMem(Line + 1 + (MaxLength - Length) / 2, Message , Length * sizeof (CHAR16));
		} else {
			// Length > MaxLength
			CopyMem(Line + 1, Message + (Length - MaxLength) / 2 , MaxLength * sizeof (CHAR16));
		}
		Line[0]             = BOXDRAW_VERTICAL;
		Line[MaxLength + 1] = BOXDRAW_VERTICAL;
		Line[MaxLength + 2] = L'\0';
		ConOut->SetCursorPosition(ConOut, Column, Row++);
		ConOut->OutputString(ConOut, Line);
		NumberOfLines--;
	}

	// Draw bottom of popup box
	SetMem16(Line, (MaxLength + 2) * 2, BOXDRAW_HORIZONTAL);
	Line[0]             = BOXDRAW_UP_RIGHT;
	Line[MaxLength + 1] = BOXDRAW_UP_LEFT;
	Line[MaxLength + 2] = L'\0';
	ConOut->SetCursorPosition(ConOut, Column, Row++);
	ConOut->OutputString(ConOut, Line);

	// Free the allocated line buffer
	FreePool (Line);

	// Restore the cursor visibility, position, and attributes
	ConOut->EnableCursor(ConOut, SavedConsoleMode.CursorVisible);
	ConOut->SetCursorPosition(ConOut, SavedConsoleMode.CursorColumn, SavedConsoleMode.CursorRow);
	ConOut->SetAttribute(ConOut, SavedConsoleMode.Attribute);
}

BOOLEAN
MoveCursor(
	IN UINT16		Direction,
	IN UINTN		EepromLength,
	IN OUT UINTN	*Column,
	IN OUT UINTN	*Row,
	IN OUT UINTN	*BaseOffset
	)
{
	UINTN	Left;
	UINTN	Right;
	UINTN	Top;
	UINTN	Bottom;
	BOOLEAN	Refresh;
	BOOLEAN	Again;

	Left = EEPROM_AREA_DATA_COL;
	Right = Left + EEPROM_ONE_ROW_MAX_LEN * 3 - 2; 
	Top = EEPROM_AREA_DATA_ROW;
	Bottom =  Top + mMaxRow - 1;
	Refresh = FALSE;
	Again = TRUE;

	do {
		Again = FALSE;  
		switch (Direction) {
		case SCAN_LEFT:
			if (*Column > Left) {
				--(*Column);
			} else {
				*Column = Right;
				Direction = SCAN_UP;
				Again = TRUE;
			}

			if (((*Column - Left) % 3) == 2) {
				--(*Column);
			}
			break;

		case SCAN_RIGHT:
			if (*Column < Right) {
				++(*Column);  
			} else {
				*Column = Left;
				Direction = SCAN_DOWN;
				Again = TRUE;
			}

			if (((*Column - Left) % 3) == 2) {
				++(*Column);
			}
			break;

		case SCAN_UP:
			if (*Row > Top) {
				--(*Row);
			} else if (*BaseOffset > 0) {
				*BaseOffset -= EEPROM_ONE_ROW_MAX_LEN;
				Refresh = TRUE;
			}
			break;

		case SCAN_DOWN:
			if (*Row < Bottom) {
				++(*Row);
			} else if ((EepromLength - (*BaseOffset)) > EEPROM_ONE_PAGE_MAX_LEN) {
				*BaseOffset += EEPROM_ONE_ROW_MAX_LEN;
				Refresh = TRUE;
			}
			break;

		case SCAN_HOME:
			*Column = Left;
			break;
        
		case SCAN_END:
			*Column = Right;
			break;
        
		case SCAN_PAGE_UP:
			if (*BaseOffset < EEPROM_ONE_PAGE_MAX_LEN) {
				*BaseOffset = 0;
			} else {
				*BaseOffset -= EEPROM_ONE_PAGE_MAX_LEN;
			}
			Refresh = TRUE;
			break;
        
		case SCAN_PAGE_DOWN:
			if ((*BaseOffset + EEPROM_ONE_PAGE_MAX_LEN) < (EepromLength - EEPROM_ONE_PAGE_MAX_LEN)) {
				*BaseOffset += EEPROM_ONE_PAGE_MAX_LEN;
			} else {
				*BaseOffset = EepromLength - EEPROM_ONE_PAGE_MAX_LEN;
			}
			Refresh = TRUE;
			break;
		}

	} while (Again);

	return Refresh;   
}

VOID
GetNetworkControllerName(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN UINTN						NetworkNameLength,
	OUT CHAR16						*NetworkControllerName
	)
{
	EFI_MAC_ADDRESS	MacAddr;

	DeviceCmds->GetMacAddress(DeviceCmds, &MacAddr);
	UnicodeSPrint(
		NetworkControllerName,
		NetworkNameLength * sizeof (CHAR16),
		L"%s (%02x:%02x:%02x:%02x:%02x:%02x)",
		DeviceCmds->GetDeviceName(DeviceCmds),
		(UINTN)MacAddr.Addr[0],
		(UINTN)MacAddr.Addr[1],
		(UINTN)MacAddr.Addr[2],
		(UINTN)MacAddr.Addr[3],
		(UINTN)MacAddr.Addr[4],
		(UINTN)MacAddr.Addr[5]
		);
}

VOID
PrintTitle(
	VOID
	)
{
	gST->ConOut->SetCursorPosition(gST->ConOut, TITLE_COLUMN, TITLE_ROW);
	Print (L" ** LAN7800 Configuration Utility **\n");
}

VOID
PrintCurrentDevice(
  IN CHAR16		*NetworkControllerName
  )
{
	gST->ConOut->SetCursorPosition(gST->ConOut, CURRENT_DEVICE_COLUMN, CURRENT_DEVICE_ROW);
	Print(L"Current Device: %s\n", NetworkControllerName);
}

VOID
PrintNetworkControllerList(
	IN EFI_HANDLE   *HandleBuffer,
	IN UINTN        HandleCount
	)
{
	EFI_STATUS					Status;
	UINTN						Index;
	CHAR16						NetworkControllerName[0x40];
	LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds;
  
	gST->ConOut->SetCursorPosition(gST->ConOut, 0, 2);
	Print (L"Number of Supported LAN78xx controller(s): %d\n", HandleCount);
  
	for (Index = 0; Index < HandleCount; ++Index) {
		Status = gBS->HandleProtocol (
				HandleBuffer[Index],
				&gLan7800DeviceCmdProtocolGuid,
				&DeviceCmds
				);
		if (EFI_ERROR (Status)) {
			continue;
		}

		GetNetworkControllerName(DeviceCmds, 0x40, NetworkControllerName);
		Print(L"  [%2d] %s\n", Index, NetworkControllerName);
	} 
}

VOID
PrintConfigFunctionList(
	IN CHAR16	*NetworkControllerName,
	OUT UINTN	*FunctionCount
	)
{
	UINTN	Index;

	PrintCurrentDevice(NetworkControllerName);
	Print(L"\n");

	for (Index = 0; mConfigFuncList[Index].ConfigFunction != NULL; ++Index) {
		Print(L"  [%2d] %s\n", Index, mConfigFuncList[Index].Description);
	}

	*FunctionCount = Index;  
}

VOID
NetworkControllerConfig(
	IN EFI_HANDLE	NetworkControllerHandle
	)
{
	EFI_STATUS					Status;
	CHAR16						NetworkControllerName[TEMP_STRING_LENGTH];
	CHAR16						Temp[TEMP_STRING_LENGTH];
	LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds;
	BOOLEAN						PressEnter;
	UINTN						Select;
	UINTN						FunctionCount;

	Status = gBS->HandleProtocol(
				NetworkControllerHandle,
				&gLan7800DeviceCmdProtocolGuid,
				&DeviceCmds
				);

	if (EFI_ERROR(Status)) {
		return;
	}

	GetNetworkControllerName(DeviceCmds, TEMP_STRING_LENGTH, NetworkControllerName);
  
	// USB Network Configuration loop
	while (TRUE) {
		gST->ConOut->ClearScreen(gST->ConOut);
		PrintTitle();

		PrintConfigFunctionList(NetworkControllerName, &FunctionCount);
		Print(L"\n");

		// Ask user to select one network controller
		Input(L"Please select function: ", Temp, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);

		// Is ESC? ESC means exit
		if (!PressEnter) {
			gST->ConOut->ClearScreen(gST->ConOut);
			break;
		}

		// Check out of range ?
		Select = StrDecimalToUintn(Temp);
		if (Select < FunctionCount) {
			mConfigFuncList[Select].ConfigFunction(DeviceCmds, NetworkControllerName);
      
		} else {
			CreatePopUp (
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Out Of Range! Please select again.  ",
			NULL
			);
		}

		gST->ConOut->ClearScreen(gST->ConOut);
		ZeroMem(Temp, 0x40 * sizeof (CHAR16));
	};
}

EFI_STATUS
EFIAPI
Lan7800UtilityMain(
	IN EFI_HANDLE        ImageHandle,
	IN EFI_SYSTEM_TABLE  *SystemTable
	)
{
	EFI_STATUS		Status;
	EFI_HANDLE		*HandleBuffer;
	UINTN			HandleCount;
	CHAR16			Temp[TEMP_STRING_LENGTH];
	BOOLEAN			PressEnter;
	UINTN			Select;

	gST->ConOut->ClearScreen(gST->ConOut);

	// Search all LAN7800 controller
	Status = gBS->LocateHandleBuffer(
				ByProtocol,
				&gLan7800DeviceCmdProtocolGuid,
				NULL,
				&HandleCount,
				&HandleBuffer
				);

	if (EFI_ERROR(Status)) {
		PrintTitle();
		Print(L"There are no supported LAN7800 controller.\n");
		return EFI_SUCCESS;
	}

	// Main loop
	while (TRUE) {

		// Print title and LAN7800s
		PrintTitle();
		PrintNetworkControllerList(HandleBuffer, HandleCount);
		Print(L"\n");

		// Ask user to select perticular LAN7800 controller
		Input(L"Please select LAN78xx controller: ", Temp, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);
		// Is ESC? ESC means exit
		if (!PressEnter) {
			gST->ConOut->ClearScreen(gST->ConOut);
			break;
		}

		// Check out of range ?
		Select = StrDecimalToUintn(Temp);
		if (Select < HandleCount) {
			NetworkControllerConfig(HandleBuffer[Select]);
		} else {
			CreatePopUp(
				EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
				&mKey,
				L"  Out Of Range! Please select again.  ",
				NULL
				);      
		}

		gST->ConOut->ClearScreen(gST->ConOut);
		ZeroMem(Temp, TEMP_STRING_LENGTH * sizeof (CHAR16));
	}

	FreePool(HandleBuffer);
  
	return EFI_SUCCESS; 
}


VOID
UpdateEepromBufferScreen(
	IN UINT8*	CurrEeprom,
	IN CHAR16	Char,
	IN UINTN	Column,
	IN UINTN	Row,
	IN UINTN	BaseOffset
	)
{
	UINTN	Offset;
	UINT8	Value;
	CHAR16	Temp[2];


	Temp[0] = Char;
	Temp[1] = 0;
	Value = (UINT8)StrHexToUintn(Temp);

	Column -= EEPROM_AREA_DATA_COL;
	Row -= EEPROM_AREA_DATA_ROW;
	Offset = BaseOffset + (Column / 3) + Row * EEPROM_ONE_ROW_MAX_LEN;

	// Update buffer
	if (Column % 3) {
		// Low
		CurrEeprom[Offset] = (Value) | (CurrEeprom[Offset] & 0xF0);
	} else {
		// High
		CurrEeprom[Offset] = (Value << 4) | (CurrEeprom[Offset] & 0x0F);
	}

	// Update screen
	if (0x60 < Char && Char < 0x7B) {
		Char -= 32;
	}
	Print (L"%c", Char);
	gST->ConOut->SetCursorPosition (gST->ConOut, EEPROM_AREA_CHAR_COL + (Column / 3), EEPROM_AREA_DATA_ROW + Row);
	Char = (CHAR16)CurrEeprom[Offset];
	if (0x7E < Char || Char < 0x20) {
		Char = L'.';
	}
	Print(L"%c", Char);
}


VOID
PrintEeprom(
	IN UINT8	*EepromBuffer,
	IN UINTN	EepromLength,
	IN UINTN	Offset
	)
{ 
	UINTN	Row;
	UINTN	Column;
	UINTN	TotalLength;
	UINTN	MaxRow;
	UINTN	MaxColumn;
	CHAR16	Char;
  
	// Initial base value
	Offset &= 0xFFFFFFF0;
	if ((EepromLength - Offset) > EEPROM_ONE_PAGE_MAX_LEN) {
		TotalLength = EEPROM_ONE_PAGE_MAX_LEN;
	} else {
		TotalLength = EepromLength - Offset;
	}
	MaxRow = TotalLength / EEPROM_ONE_ROW_MAX_LEN;
	mMaxRow = MaxRow;
	MaxColumn = EEPROM_ONE_ROW_MAX_LEN;
  
	// Print column offset title
	gST->ConOut->SetCursorPosition (gST->ConOut, EEPROM_AREA_DATA_COL, EEPROM_AREA_BASE_ROW);
	for (Column = 0; Column < MaxColumn; ++Column) {
		Print (L"%02x ", Column);
	}

	// Print row offset title and EEPROM data line by line
	for (Row = 0; Row < MaxRow; ++Row) { 
		// Set position and print row offset title
		gST->ConOut->SetCursorPosition (gST->ConOut, EEPROM_AREA_BASE_COL, EEPROM_AREA_DATA_ROW + Row);
		Print (L"%04x ", Row * EEPROM_ONE_ROW_MAX_LEN + Offset);

		// EEPROM data
		for (Column = 0; Column < MaxColumn; ++Column) {
			Print (L"%02x ", EepromBuffer[Row * EEPROM_ONE_ROW_MAX_LEN + Column + Offset]);
		}

		// EEPROM char
		gST->ConOut->SetCursorPosition (gST->ConOut, EEPROM_AREA_CHAR_COL, EEPROM_AREA_DATA_ROW + Row);
		for (Column = 0; Column < MaxColumn; ++Column) {
			Char = (CHAR16)EepromBuffer[Row * EEPROM_ONE_ROW_MAX_LEN + Column + Offset];
			if (0x7E < Char || Char < 0x20) {
				Char = L'.';
			}
			Print (L"%c", Char);
		}
		Print (L"\n");
	} 
}

VOID
UpdateEeprom(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN UINT8						*CurrEeprom,
	IN UINT8						*OrigEeprom,
	IN UINTN						EepromLength
	)
{
	UINTN		Index;
	EFI_STATUS	Status = EFI_SUCCESS;
	CHAR16		TempString[TEMP_STRING_LENGTH];
  
	// No change
	if (CompareMem(CurrEeprom, OrigEeprom, EepromLength) == 0) {
		return;
	}

	// Make sure user want to save change
	gST->ConOut->ClearScreen(gST->ConOut);
	gST->ConOut->EnableCursor(gST->ConOut, FALSE);
	CreatePopUp(
		EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
		&mKey,
		L"  Do you want to save change to EEPROM? (Y/N)  ",
		NULL
		);

	if (mKey.UnicodeChar != L'Y' && mKey.UnicodeChar != L'y') {
		goto EXIT;
	}

	gST->ConOut->ClearScreen(gST->ConOut);
	gST->ConOut->EnableCursor(gST->ConOut, FALSE);
	CreatePopUp (
		EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
		&mKey,
		L" Are you sure? (Y/N) ",
		NULL
		);

	if (mKey.UnicodeChar != L'Y' && mKey.UnicodeChar != L'y') {
		goto EXIT;
	}

	// Update
	for (Index = 0; Index < EepromLength; ++Index) {
		if (CurrEeprom[Index] != OrigEeprom[Index]) {
			//Print (L"Update Offset 0x%02x to %02x\n", Index, CurrEeprom[Index]);
			Status = DeviceCmds->WriteEeprom(DeviceCmds, Index, 1, &CurrEeprom[Index]);
			if (EFI_ERROR (Status)) {
				break;
			}
		}
	}

	gST->ConOut->ClearScreen(gST->ConOut);
	gST->ConOut->EnableCursor(gST->ConOut, FALSE);
	UnicodeSPrint(TempString, TEMP_STRING_LENGTH * sizeof (CHAR16), L"  Updaet EEPROM Finish! Status: %r  ", Status);
	CreatePopUp(
		EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
		&mKey,
		TempString,
		NULL
		);
EXIT:
	gST->ConOut->EnableCursor(gST->ConOut, TRUE); 
}

VOID
PrintEepromHelp(
	VOID
	)
{
	gST->ConOut->SetCursorPosition(gST->ConOut, EEPROM_AREA_HELP_COL, EEPROM_AREA_HELP_ROW);
	Print (L"Use [ESC] to exit EEPROM editor.");
	gST->ConOut->SetCursorPosition(gST->ConOut, EEPROM_AREA_HELP_COL, EEPROM_AREA_HELP_ROW + 1);
	Print (L"Use [Up][Down][Left][Right][Home][End][PageUp][PageDown] to move cursor.");
}

EFI_STATUS
ViewEditEeprom(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	)
{
	UINT8		*OrigEeprom;
	UINT8		*CurrEeprom;
	EFI_STATUS	Status;
	UINT32		EepromLength = 0;
	UINTN		EventIndex;
	BOOLEAN		Refresh;
	UINTN		CurrColumn;
	UINTN		CurrRow;
	UINTN		CurrBaseOffset;
	BOOLEAN	Done;

	gST->ConOut->ClearScreen (gST->ConOut);

	// Initial local variable
	Status = EFI_SUCCESS;
	CurrEeprom = NULL;
	OrigEeprom = NULL;

	Status = DeviceCmds->GetEepromLength(DeviceCmds, &EepromLength);
	if (Status == EFI_UNSUPPORTED || Status == EFI_NOT_FOUND) {
		CreatePopUp (
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Cannot Find EEPROM.  ",
			NULL
			);

		return EFI_NOT_FOUND;
	}

	// Allocate memory for EEPROM
	OrigEeprom = AllocateZeroPool(EepromLength);
	if (OrigEeprom == NULL) {
		CreatePopUp (
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Cannot Allocate Memory for EEPROM.  ",
			NULL
			);

		Status = EFI_OUT_OF_RESOURCES;
		goto EXIT;
	}

	CurrEeprom = AllocateZeroPool(EepromLength);
	if (CurrEeprom == NULL) {
		CreatePopUp (
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Cannot Allocate Memory for EEPROM.  ",
			NULL
		);

		Status = EFI_OUT_OF_RESOURCES;
		goto EXIT;
	}

	// For slow device, pop a message to let user know we're reading EEPROM
	MessagePopUp(EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK, L"  Reading EEPROM ...  ");
  
	Status = DeviceCmds->ReadEeprom(DeviceCmds, 0, EepromLength, OrigEeprom);
	if (EFI_ERROR (Status)) {
		gST->ConOut->ClearScreen(gST->ConOut);
		CreatePopUp (
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Read EEPROM Fail.  ",
			NULL
			);
		goto EXIT;
	}
	CopyMem(CurrEeprom, OrigEeprom, EepromLength);

	// Print title and device name then enter Edit EEPROM main loop
	PrintTitle();
	PrintCurrentDevice(NetworkControllerName);
	PrintEepromHelp();
	Refresh = TRUE;
	CurrRow = EEPROM_AREA_DATA_ROW;
	CurrColumn = EEPROM_AREA_DATA_COL;
	CurrBaseOffset = 0;
	Done = FALSE;
 
	while (!Done) {
		// No need to updaet screen every time
		if (Refresh) {
			PrintEeprom(CurrEeprom, EepromLength, CurrBaseOffset);
		}
		Refresh = FALSE;

		gST->ConOut->SetCursorPosition(gST->ConOut, CurrColumn, CurrRow);

		// Wait key
		gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
		gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);

		// Parse key
		switch (mKey.UnicodeChar) {
		// Change cursor position, ESC
		case CHAR_NULL:
			switch (mKey.ScanCode) {
			// Exit
			case SCAN_ESC:
				Done = TRUE;
				break;
          
			// Change cursor position
			case SCAN_LEFT:
			case SCAN_RIGHT:
			case SCAN_UP:
			case SCAN_DOWN:
			case SCAN_HOME:
			case SCAN_END:
			case SCAN_PAGE_UP:
			case SCAN_PAGE_DOWN:
				Refresh = MoveCursor(mKey.ScanCode, EepromLength, &CurrColumn, &CurrRow, &CurrBaseOffset);
				break;  
			}
			break;

		// Normal character
		default:
			if (IsHexDigitChar (mKey.UnicodeChar)) {
				UpdateEepromBufferScreen(CurrEeprom, mKey.UnicodeChar, CurrColumn, CurrRow, CurrBaseOffset);
				Refresh = MoveCursor(SCAN_RIGHT, EepromLength, &CurrColumn, &CurrRow, &CurrBaseOffset);
			}
			break; 
		}
	}

	// Here means user press ESC
	UpdateEeprom(DeviceCmds, CurrEeprom, OrigEeprom, EepromLength);

EXIT:
	if (CurrEeprom != NULL) {
		FreePool (CurrEeprom);
	}

	if (OrigEeprom != NULL) {
		FreePool (OrigEeprom);
	}
  
	return Status;
}

EFI_STATUS
ViewProperty(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	)
{
	UINT32		Duplex;
	UINT32		LinkSpeed;
	UINTN		EventIndex;
	EFI_STATUS	Status = EFI_SUCCESS;

	gST->ConOut->ClearScreen(gST->ConOut);
	PrintTitle();
	PrintCurrentDevice(NetworkControllerName);
	Print(L"\n");

	Status = DeviceCmds->GetLinkStatus(DeviceCmds, &LinkSpeed, &Duplex);
	if (EFI_ERROR(Status)) {
		return EFI_DEVICE_ERROR;
	}
	
	if (LinkSpeed == SPEED_0) {
		Print(L"No Link\n");
	} else if (LinkSpeed == SPEED_10) {
		Print(L"10 Mbps,");
	} else if (LinkSpeed == SPEED_100) {
		Print(L"100 Mbps,");
	} else if (LinkSpeed == SPEED_1000) {
		Print(L"1000 Mbps,");
	}
	if (Duplex == HALF_DUPLEX) {
		Print(L" Half-Duplex\n");
	} else if (Duplex == FULL_DUPLEX) {
		Print(L" Full-Duplex\n");
	}
	Print(L"\n");
	Print(L"Press Any Key to Continue...");
	gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
	gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
	return Status;
}

EFI_STATUS
ViewEditPhyRegister(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	)
{
	EFI_STATUS             Status;
	UINT32                 Data;
	UINTN                  MiiRegister;
	UINTN                  EventIndex;
	CHAR16                 TempString[TEMP_STRING_LENGTH];
	BOOLEAN                PressEnter;

	while (TRUE) { 
		gST->ConOut->ClearScreen(gST->ConOut);

		PrintTitle();
		PrintCurrentDevice(NetworkControllerName);
		Print(L"\n");

		ZeroMem(TempString, TEMP_STRING_LENGTH * sizeof (CHAR16));
    
		Input(L"Please input phy register: 0x", TempString, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);

		if (!PressEnter) {
			break;
		}
		Print(L"\n");

		MiiRegister = StrHexToUintn(TempString);

		Status = DeviceCmds->ReadPhy(DeviceCmds, 1, (UINT32)MiiRegister, &Data);
		if (EFI_ERROR (Status)) {
			// Wait key
			Print (L"Phy register: %r\n", Status);
			Print (L"\n");
			Print (L"Press Any Key to Continue...");
			gBS->WaitForEvent (1, &gST->ConIn->WaitForKey, &EventIndex);
			gST->ConIn->ReadKeyStroke (gST->ConIn, &mKey);

			break;
		}

		Print(L"Phy register 0x%02x: 0x%08x\n", MiiRegister, Data);
		Print(L"\n");
		Print(L"Do you want to modify register 0x%02x? (y/n)\n", MiiRegister);
		gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
		gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
		if (mKey.UnicodeChar != L'Y' && mKey.UnicodeChar != L'y') {
			continue;
		}

		Print(L"Please input new value of Phy register 0x%02x: 0x", MiiRegister);

		ZeroMem(TempString, TEMP_STRING_LENGTH * sizeof (CHAR16));
		Input(NULL, TempString, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);

		if (PressEnter && TempString[0] != 0) {
			Print(L"\n");
			Print(L"\n");
      
			Data = (UINT32)StrHexToUintn(TempString);
			Status = DeviceCmds->WritePhy(DeviceCmds, 1, (UINT32)MiiRegister, Data);
			Print(L"Write 0x%08x to Phy register 0x%02x: %r\n", Data, MiiRegister, Status);

			// Wait key
			Print(L"\n");
			Print(L"Press Any Key to Continue...");
			gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
			gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
		}   
	}  
	return EFI_SUCCESS;
}


EFI_STATUS
ViewEditRegister(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	)
{
	EFI_STATUS	Status;
	UINT32		Data;
	UINT16		Register;
	UINTN		EventIndex;
	CHAR16		TempString[TEMP_STRING_LENGTH];
	BOOLEAN		PressEnter;

	while (TRUE) { 
		gST->ConOut->ClearScreen(gST->ConOut);

		PrintTitle();
		PrintCurrentDevice(NetworkControllerName);
		Print (L"\n");

		ZeroMem(TempString, TEMP_STRING_LENGTH * sizeof (CHAR16));
    
		Input(L"Please input device register: 0x", TempString, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);

		if (!PressEnter) {
			break;
		}
		Print(L"\n");

		Register = (UINT16) StrHexToUintn(TempString);

		Status = DeviceCmds->ReadRegister(DeviceCmds, Register, &Data);
		if (EFI_ERROR(Status)) {
			// Wait key
			Print(L"Register: %r\n", Status);
			Print(L"\n");
			Print(L"Press Any Key to Continue...");
			gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
			gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
			break;
		}

		Print(L"Register 0x%02x: 0x%08x\n", Register, Data);
		Print(L"\n");
		Print(L"Do you want to modify register 0x%02x? (y/n)\n", Register);
		gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
		gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
		if (mKey.UnicodeChar != L'Y' && mKey.UnicodeChar != L'y') {
			continue;
		}

		Print(L"Please input new value of device register 0x%02x: 0x", Register);

		ZeroMem (TempString, TEMP_STRING_LENGTH * sizeof (CHAR16));
		Input (NULL, TempString, TEMP_STRING_LENGTH, IsHexDigitChar, &PressEnter, NULL);

		if (PressEnter && TempString[0] != 0) {
			Print (L"\n");
			Print (L"\n");
      
			Data = (UINT32)StrHexToUintn(TempString);
			Status = DeviceCmds->WriteRegister(DeviceCmds, (UINT32)Register, Data);
			Print(L"Write 0x%08x to device register 0x%02x: %r\n", Data, Register, Status);

			// Wait key
			Print(L"\n");
			Print(L"Press Any Key to Continue...");
			gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
			gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);
		}  
	}  

	return EFI_SUCCESS;
}

EFI_STATUS
DumpOtp(
	IN LAN7800_DEVICE_CMD_PROTOCOL	*DeviceCmds,
	IN CHAR16						*NetworkControllerName
	)
{
	UINTN		EventIndex;
	UINT8		*OtpBuffer = NULL;
	EFI_STATUS	Status = EFI_SUCCESS;

	gST->ConOut->ClearScreen(gST->ConOut);
  
	// Allocate memory for otp
	OtpBuffer = AllocateZeroPool(512);
	if (OtpBuffer == NULL) {
		CreatePopUp(
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Cannot Allocate Memory for Otp.  ",
			NULL
			);

		Status = EFI_OUT_OF_RESOURCES;
		return Status;
	}

	// For slow device, pop a message to let user know we're reading otp
	MessagePopUp(EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK, L"  Reading Otp ...  ");
  
	Status = DeviceCmds->DumpOtp(DeviceCmds, 0, 512, OtpBuffer);
	if (EFI_ERROR(Status)) {
		gST->ConOut->ClearScreen(gST->ConOut);
		CreatePopUp(
			EFI_LIGHTGRAY | EFI_BACKGROUND_BLACK,
			&mKey,
			L"  Read Otp Fail.  ",
			NULL
			);
		Status = EFI_OUT_OF_RESOURCES;
		return Status;
	}

	// Print title and device name then enter Edit EEPROM main loop
	PrintTitle();
	PrintCurrentDevice(NetworkControllerName);
  
	PrintEeprom(OtpBuffer, 512, 0);
	Print(L"Press Any Key to Continue...");
	gBS->WaitForEvent(1, &gST->ConIn->WaitForKey, &EventIndex);
	gST->ConIn->ReadKeyStroke(gST->ConIn, &mKey);

	return Status;
}

