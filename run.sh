#!/bin/bash
echo "BASH install_ubuntu start!"

# Set the current directory to where the script is located
cd "$(dirname "$0")"

# Navigate to the cmake directory and create a build directory
cd build

# Build the wood code after making changes (e.g., in main.cpp)
cd "$(dirname "$0")"/build

# Step 7: Build the nest code
cmake -B . -S .. -DGET_LIBS=OFF -DBUILD_MY_PROJECTS=ON -DCOMPILE_LIBS=OFF -DRELEASE_DEBUG=ON -DCMAKE_BUILD_TYPE="Release" -G "Unix Makefiles" && make -j$(sysctl -n hw.ncpu)

# Step 8: Build the nest code after making changes (e.g., in main.cpp)
make VERBOSE=1 && ../build/nest

cd ../..

echo "BASH install_ubuntu end!"
