#!/bin/bash
export PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
cd "$(dirname "$0")" || exit 1

echo "NS26F Telemetry Dashboard: http://localhost:8081"
open "http://localhost:8081"
python3 -m http.server 8081
