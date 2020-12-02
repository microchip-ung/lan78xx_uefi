# lan78xx_uefi
LAN7800 UEFI driver

Step for Build Lan7800UndiPkg

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
