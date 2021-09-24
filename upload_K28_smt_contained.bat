@echo off
echo Bootload ATMega328p and upload K28 Surface Mount code
echo PCB Version: 2
echo Property of LRRD - Copyright 2021
echo Written by Mason Parrone
echo Programming tool: USBtinyISP
echo Connect board using 6 pin ICSP header
echo Once it is in place, press a key to continue
pause
echo Bootloading...
REM avrdude takes care of the work
REM point to config file found in: 
avrdude -Cavrdude.conf -v -patmega328p -cusbtiny -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDA:m -Ulfuse:w:0x62:m 
pause
echo Uploading Code...
avrdude -Cavrdude.conf -v -patmega328p -cusbtiny -Uflash:w:K28_Code_v2.ino.eightanaloginputs.hex :i 
pause