@echo off

echo BAT install_windows start!

REM Set the current directory to where the batch file is located
cd /d %~dp0

REM Clone the repository only if it doesn't already exist
if not exist ..\nest git clone https://github.com/petrasvestartas/nest.git

REM Navigate to the cmake directory and create a build directory
cd cmake\
if not exist build mkdir build
cd build

REM Step 5: Download libraries
cmake --fresh -DGET_LIBS=ON -DCOMPILE_LIBS=OFF -DBUILD_MY_PROJECTS=OFF -DRELEASE_DEBUG=ON -DCMAKE_BUILD_TYPE="Release" -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release

REM Step 6: Build 3rd-party libraries
cmake --fresh -DGET_LIBS=OFF -DBUILD_MY_PROJECTS=ON -DCOMPILE_LIBS=ON -DRELEASE_DEBUG=ON -DCMAKE_BUILD_TYPE="Release" -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release

REM Step 7: Build the wood code
cmake --fresh -DGET_LIBS=OFF -DBUILD_MY_PROJECTS=ON -DCOMPILE_LIBS=OFF -DRELEASE_DEBUG=ON -DCMAKE_BUILD_TYPE="Release" -G "Visual Studio 17 2022" -A x64 .. && cmake --build . --config Release

REM Step 8: Build the wood code after making changes (e.g., in main.cpp)
cd /d %~dp0\cmake\build
cmake --build . -v --config Release --parallel 8 && ..\build\Release\nest.exe
cd ..\..

echo BAT install_windows end!
pause