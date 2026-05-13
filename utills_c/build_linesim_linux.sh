#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
gcc -shared -fPIC -O2 -o liblinesim.so linesim.c -lm
