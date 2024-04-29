@echo off
echo BAT compile and run start!

REM Set the current directory to where the batch file is located
cd /d %~dp0

REM Clone the repository only if it doesn't already exist
if not exist ..\wood_cpp git clone https://github.com/petrasvestartas/nest.git

REM Navigate to the cmake directory and create a build directory
cd cmake\
cd build\

REM Step 8: Build the wood code after making changes (e.g., in main.cpp)
cd /d %~dp0\cmake\build
cmake  --build C:\brg\2_code\nest\cmake\build -v --config Release --parallel 8 &&  C:\brg\2_code\nest\cmake\build\Release\nest.exe
cd ..\..

echo BAT compile and run end!


