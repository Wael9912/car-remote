#!/bin/bash
# Print the car's trip log over USB (the Arduino keeps it in EEPROM between runs).
P=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
[ -z "$P" ] && { echo "Arduino not connected"; exit 1; }
OUT=$(mktemp)
stty -f "$P" 9600 raw
exec 3<>"$P"            # opening the port restarts the Arduino
cat <&3 > "$OUT" & CAT=$!
sleep 3.5               # boot + servo glance
printf G >&3
for i in $(seq 30); do grep -q "LOG END" "$OUT" && break; sleep 0.5; done
kill $CAT; wait $CAT 2>/dev/null; exec 3>&-
tr -d '\0\r' < "$OUT" | sed -n '/LOG BEGIN/,/LOG END/p'
rm -f "$OUT"
