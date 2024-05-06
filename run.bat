@echo off
echo BAT compile and run start!

REM Set the current directory to where the batch file is located
cd /d %~dp0


REM Navigate to the cmake directory and create a build directory
cd build\

REM Step 8: Build the wood code after making changes (e.g., in main.cpp)
cd /d %~dp0\cmake\build
cmake  --build C:\brg\2_code\nest\build -v --config Release --parallel 8 &&  C:\brg\2_code\nest\build\Release\nest.exe
cd ..\..

echo BAT compile and run end!


