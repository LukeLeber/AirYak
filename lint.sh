#!/bin/bash
set -euo pipefail

# Create an isolated temporary directory in /tmp.
TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/airyak-lint.XXXXXX")

# Always clean up the temporary directory on exit.
cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT

# Generate compile_commands.json directly inside /tmp.
arduino-cli compile \
    --fqbn arduino:avr:uno \
    --libraries ./lib \
    --only-compilation-database \
    --build-path "$TMP_DIR" \
    .

if [ ! -f "$TMP_DIR/compile_commands.json" ]; then
    echo "Error: compile_commands.json was not generated." >&2
    exit 1
fi

# Point clang-tidy directly at the /tmp build folder.
find ./lib -type f -name '*.cpp' -print0 | xargs -0 clang-tidy -p "$TMP_DIR"