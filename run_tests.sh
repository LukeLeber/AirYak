#!/bin/bash
set -euo pipefail

# Configuration
TEST_DIR="./tests"
LIB_DIR="./lib"
BUILD_DIR=$(mktemp -d "${TMPDIR:-/tmp}/airyak-tests.XXXXXX")
FAILED=0
PASSED_COUNT=0
FAILED_COUNT=0

# Always remove temporary build artifacts on exit
cleanup() {
    rm -rf "$BUILD_DIR"
}
trap cleanup EXIT

echo "===================================================="
echo "      🚀 Running AirYak Native Component Tests       "
echo "===================================================="

# Gather all tests.
mapfile -t TEST_FILES < <(find "$TEST_DIR" -type f -name "*.ino" | sort)

if [ ${#TEST_FILES[@]} -eq 0 ]; then
    echo "❌ No component test files (*.ino) found in $TEST_DIR." >&2
    exit 1
fi

ABS_MOCK_DIR="$(cd "$TEST_DIR/mock_arduino" && pwd)"
ABS_TEST_DIR="$(cd "$TEST_DIR" && pwd)"
SUBMODULE_UNITY_DIR="$ABS_TEST_DIR/unity_framework/src"

if [ ! -d "$SUBMODULE_UNITY_DIR" ]; then
    echo "❌ Unity submodule code not found at $SUBMODULE_UNITY_DIR" >&2
    echo "   Please execute: git submodule update --init --recursive" >&2
    exit 1
fi

# Build shared include paths across mock headers, Unity, and all /lib modules.
INCLUDE_FLAGS=("-I$ABS_MOCK_DIR" "-I$ABS_TEST_DIR" "-I$SUBMODULE_UNITY_DIR")
for lib_sub_dir in "$LIB_DIR"/*; do
    if [ -d "$lib_sub_dir" ]; then
        ABS_LIB_SUB_DIR="$(cd "$lib_sub_dir" && pwd)"
        INCLUDE_FLAGS+=("-I$ABS_LIB_SUB_DIR")
    fi
done

# Pre-compile Unity, mock Arduino, and library objects once for fast linking.
SHARED_OBJS=()
gcc -c "$SUBMODULE_UNITY_DIR/unity.c" -I"$SUBMODULE_UNITY_DIR" -o "$BUILD_DIR/unity.o"
SHARED_OBJS+=("$BUILD_DIR/unity.o")

g++ -std=c++11 -Wall -Wextra -c "$TEST_DIR/mock_arduino/Arduino.cpp" "${INCLUDE_FLAGS[@]}" -o "$BUILD_DIR/Arduino.o"
SHARED_OBJS+=("$BUILD_DIR/Arduino.o")

while IFS= read -r src_file; do
    obj_name="$(basename "${src_file%.*}").o"
    g++ -std=c++11 -Wall -Wextra -c "$src_file" "${INCLUDE_FLAGS[@]}" -o "$BUILD_DIR/$obj_name"
    SHARED_OBJS+=("$BUILD_DIR/$obj_name")
done < <(find "$LIB_DIR" -type f -name "*.cpp")

# Compile and run each .ino test suite against the pre-compiled objects.
for test_file in "${TEST_FILES[@]}"; do
    test_name=$(basename "$test_file")
    component_name=$(echo "$test_name" | sed -E 's/Test\.ino$//I')
    binary_out="$BUILD_DIR/${component_name}_runner"

    echo -n "🧪 Testing [$component_name] ... "

    if ! compile_output=$(g++ -std=c++11 -Wall -Wextra \
        "${INCLUDE_FLAGS[@]}" \
        -x c++ "$test_file" \
        -x none "${SHARED_OBJS[@]}" \
        -o "$binary_out" 2>&1); then
        echo "❌ COMPILE FAILED"
        echo "$compile_output"
        echo "----------------------------------------------------"
        FAILED=1
        ((FAILED_COUNT++)) || true
        continue
    fi

    if run_output=$("$binary_out" 2>&1); then
        echo "✅ PASSED"
        ((PASSED_COUNT++)) || true
    else
        echo "❌ FAILED"
        echo "$run_output"
        echo "----------------------------------------------------"
        FAILED=1
        ((FAILED_COUNT++)) || true
    fi
done

echo "===================================================="
echo "📊 Test Execution Summary:"
echo "   Passed:   $PASSED_COUNT"
echo "   Failed:   $FAILED_COUNT"
echo "===================================================="

if [ "$FAILED" -eq 0 ]; then
    echo "🎉 All component systems are stable!"
    exit 0
else
    echo "🚨 One or more component suites failed." >&2
    exit 1
fi