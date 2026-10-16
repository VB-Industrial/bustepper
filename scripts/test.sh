#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d /tmp/bustepper-tests.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
for test_source in tests/units.cpp tests/tmc5160.cpp; do
    test_name=$(basename "$test_source" .cpp)
    c++ -std=c++20 -Wall -Wextra -Werror -fsanitize=undefined,address \
        -ICore/Inc "$test_source" -o "$test_dir/$test_name"
    "$test_dir/$test_name"
    echo "$test_name: passed"
done
