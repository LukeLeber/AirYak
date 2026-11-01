#!/bin/bash

# Create a temporary directory for the build output
mkdir -p build

# 1. Generate the compilation database into the separate build directory
arduino-cli compile --fqbn arduino:avr:uno --libraries ./lib --only-compilation-database --build-path ./build

# 2. Move the generated database to the project root where clang-tidy expects it
if [ -f "./build/compile_commands.json" ]; then
    cp ./build/compile_commands.json .
else
    echo "Error: compile_commands.json was not generated."
    exit 1
fi

# 3. Clean up the temporary build directory
rm -rf build

# 4. Run clang-tidy using the database now located in the current folder (.)
clang-tidy -p . lib/**/*.cpp