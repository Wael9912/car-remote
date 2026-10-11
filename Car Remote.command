#!/bin/bash
# Double-click to open the car remote in Chrome.
# Bluetooth in the browser needs the page served from localhost, so this runs a tiny local web server.
cd "$(dirname "$0")"
lsof -i :8765 >/dev/null 2>&1 || (python3 -m http.server 8765 --bind 127.0.0.1 >/dev/null 2>&1 &)
sleep 1
open -a "Google Chrome" "http://localhost:8765/index.html"
