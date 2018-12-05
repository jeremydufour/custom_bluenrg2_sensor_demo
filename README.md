# custom_bluenrg2_sensor_demo
This is a BlueNRG2 demo project compatible with the armgcc toolchain

## Requirements
This code works for STEVAL-IDB008V2 development board

Version of armgcc used: 7-2017-q4-major

In order to compile the project, you have to install the armgcc toolchain on a Windows computer.
Follow those links to get all of the tools used to compile the code:

http://gnuwin32.sourceforge.net/packages/make.htm

https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

You will also have to add the BlueNRG_DK_X.X.X folder to the repository

## Recommendations
As I could not port the code to be compiled on Linux, it is a major feature to add to this project.
If you want to contribute to this feature, you can try by installing the same armgcc toolchain on you Linux environment.
The makefile has to be changed to be compatible with make on Linux.
