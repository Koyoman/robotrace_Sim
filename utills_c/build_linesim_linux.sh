#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
gcc -shared -O2 -fPIC -o liblinesim.so linesim.c -lm
echo "Built $(pwd)/liblinesim.so"
