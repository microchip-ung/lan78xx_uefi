# lan78xx_uefi
LAN7800 UEFI driver

Steps for Build Lan7800UndiPkg

0. Make sure the build machine had install MSVC2005 or MSVC2008 already.

1. Download EDK II from https://github.com/tianocore/tianocore.github.io/wiki/UDK2014-Releases

2. Follow the procedure specified in the UDK2014 to setup build environment.

3. Run "edksetup.bat --nt32 "

5. Modify TOOL_CHAIN_TAG in c:\MyWorkSpace\Conf\target.txt.  
   Win7_64Bit + MSVC2008 : TOOL_CHAIN_TAG = VS2008x86
   If your enviroment not same as this. Please reference tools_def.txt or EDK II build document.

6. Run "Build -p Lan7800UndiPkg\Lan7800UndiPkg.dsc -b RELEASE -a IA32"

7. The binary files will locate at .\MyWorkSpace\Build\Lan7800UndiPkg\RELEASE_VS2008x86\IA32.
    Driver  : Lan7800Driver.efi
    Utility : Lan7800Utility.efi

8. To build x64 binaries: Run "Build -p Lan7800UndiPkg\Lan7800UndiPkg.dsc -b RELEASE -a X64"
    The x64 binary files will be at .\MyWorkSpace\Build\Lan7800UndiPkg\RELEASE_VS2008x86\X64

=============================================================================================
If you do not have MSVC2005 or MSVC2008, then...

Building LAN78xx PXE Driver against UDK2017 with Visual Studio 2015 Express

All that is required for building the driver is access to the command line build tools. The IDE environment is not necessary.

Environment Setup
The following instructions were used to build against UDK2017:

UDK2017 Build instructions

Steps followed:

Setup Build Environment
   Added environment variables (system - due to local administrator permissions)
   NASM_PREFIX = C:\NASM\
   PYTHON_HOME = C:\Python27
   Added C:\Python27 to system path
Create the full Source Code directory for UDK2017 release
   Implemented Optional step iv instead of building the tools from source
Build Steps NT32
   Edited MyWorkspace\Conf\target.txt
   TARGET_ARCH = X64
   TOOL_CHAIN_TAG = VS2015x86
   MAX_CONCURRENT_THREAD_NUMBER = 13 (1 + number of cores)
   With the target.txt edits, the build command can be used without -t option to build HelloWorld.efi
Other
   Symantec Endpoint Protection will flag two files during the HelloWorld.efi build process. Exclude the C:\MyWorkspace directory to allow the build to succeed.

Source code changes
The libc memory routines were not able to resolve at linking. Not sure if this is a change between UDK2014 and UDK2017, but memory routines are provided in UDK2017. The source was modified to accommodate these changes via search and replace:

memcpy -> CopyMem
memset -> SetMem
memcmp -> CompareMem
Changes were limited to Lan7800Driver.c and Lan7800.c

Building the Lan7800 Driver
After testing the UDK installation, copy the Lan7800UndiPkg directory into C:\MyWorkspace

Launch command prompt from Visual Studio 2015\Launch Developer Command Prompt (from Start Menu)

cd into C:\MyWorkspace

To build the Lan7800UndiPkg:

build -p Lan7800UndiPkg\Lan7800UndiPkg.dsc
Release build:

build -p Lan7800UndiPkg\Lan7800UndiPkg.dsc -b RELEASE
Build just the driver:

build -p Lan7800UndiPkg\Lan7800UndiPkg.dsc -m Lan7800UndiPkg\Driver\Lan7800Driver.inf
The resulting efi files are located in C:\MyWorkspace\Build\Lan7800UndiPkg\DEBUG_VS2015x86\X64
