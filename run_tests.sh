#!/bin/bash

# Configuration
TEST_DIR="./tests"
LIB_DIR="./lib"
BUILD_DIR="./build_tests"
FAILED=0
PASSED_COUNT=0
FAILED_COUNT=0

# Clean and prepare workspace
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

echo "===================================================="
echo "      🚀 Running AirYak Native Component Tests       "
echo "===================================================="

# Step 1: Find all valid test files
TEST_FILES=$(find "$TEST_DIR" -type f \( -name "*_test.ino" -o -name "*_test.cpp" -o -name "test_*.ino" -o -name "test_*.cpp" \))

if [ -z "$TEST_FILES" ]; then
    echo "❌ No component test files found matching pattern (*_test or test_*)."
    exit 1
fi

# Resolve absolute tracking paths for the testing directories
ABS_MOCK_DIR="$(cd "$TEST_DIR/mock_arduino" && pwd)"
ABS_TEST_DIR="$(cd "$TEST_DIR" && pwd)"

for test_file in $TEST_FILES; do
    test_name=$(basename "$test_file")
    component_name=$(echo "$test_name" | sed -E 's/(_test|test_)\.(ino|cpp)//I')
    binary_out="$BUILD_DIR/${component_name}_runner"
    
    # Step 2: Dynamically locate the implementation file anywhere under lib/
    impl_file=$(find "$LIB_DIR" -type f -name "${component_name}.cpp" | head -n 1)
    
    if [ -z "$impl_file" ] || [ ! -f "$impl_file" ]; then
        echo "⚠️  Warning: No matching implementation found for $test_name. Compiling standalone."
        impl_file=""
        ABS_COMP_DIR=""
    else
        # Extract the absolute directory where the component lives
        ABS_COMP_DIR="$(cd "$(dirname "$impl_file")" && pwd)"
    fi

    echo -n "🧪 Testing [$component_name] ... "

    # Step 3: Map paths directly into your local Git submodule dependency
    SUBMODULE_UNITY_DIR="$ABS_TEST_DIR/unity_framework/src"
    
    if [ ! -d "$SUBMODULE_UNITY_DIR" ]; then
        echo "❌ Unity submodule code not found at $SUBMODULE_UNITY_DIR"
        echo "   Please execute: git submodule update --init --recursive"
        exit 1
    fi

    # Initialize basic testing paths
    INCLUDE_FLAGS=("-I$ABS_MOCK_DIR" -I"$ABS_TEST_DIR" -I"$SUBMODULE_UNITY_DIR")
    
    # Target individual components
    if [ -n "$ABS_COMP_DIR" ]; then
        INCLUDE_FLAGS+=("-I$ABS_COMP_DIR" "-isystem" "$ABS_COMP_DIR")
    fi

    # CRITICAL ADDITION: Loop through all subfolders in /lib and add them as system search paths
    # This resolves cross-component dependencies natively for g++
    for lib_sub_dir in "$LIB_DIR"/*; do
        if [ -d "$lib_sub_dir" ]; then
            ABS_LIB_SUB_DIR="$(cd "$lib_sub_dir" && pwd)"
            INCLUDE_FLAGS+=("-I$ABS_LIB_SUB_DIR" "-isystem" "$ABS_LIB_SUB_DIR")
        fi
    done

    # Step 3.5: Dynamically locate ALL other implementation files inside the lib/ tree
    # This ensures sibling components link correctly during multi-component tests
    OTHER_IMPL_FILES=$(find "$LIB_DIR" -type f -name "*.cpp" ! -path "$impl_file")

    # Step 4: Compile the native Linux binary using g++
    # Note that ${OTHER_IMPL_FILES} is appended to the compilation source arguments list
    g++ -std=c++11 \
        "${INCLUDE_FLAGS[@]}" \
        -x c++ "$test_file" \
        -x none "$impl_file" ${OTHER_IMPL_FILES} "$TEST_DIR/mock_arduino/Arduino.cpp" "$SUBMODULE_UNITY_DIR/unity.c" \
        -o "$binary_out" 2> "$BUILD_DIR/${component_name}_compile_error.log"
    if [ $? -ne 0 ]; then
        echo "❌ COMPILE FAILED"
        cat "$BUILD_DIR/${component_name}_compile_error.log"
        echo "----------------------------------------------------"
        FAILED=1
        ((FAILED_COUNT++))
        continue
    fi

    # Step 5: Run the test binary
    "$binary_out" > "$BUILD_DIR/${component_name}_run.log" 2>&1
    run_status=$?

    if [ $run_status -eq 0 ]; then
        echo "✅ PASSED"
        ((PASSED_COUNT++))
    else
        echo "❌ FAILED"
        cat "$BUILD_DIR/${component_name}_run.log"
        echo "----------------------------------------------------"
        FAILED=1
        ((FAILED_COUNT++))
    fi
done

echo "===================================================="
echo "📊 Test Execution Summary:"
echo "   Passed:   $PASSED_COUNT"
echo "   Failed:   $FAILED_COUNT"
echo "===================================================="

if [ $FAILED -eq 0 ]; then
    rm -rf "$BUILD_DIR"
    echo "🎉 All component systems are stable!"
    exit 0
else
    echo "🚨 One or more component suites failed. Check build logs inside $BUILD_DIR/"
    exit 1
fi