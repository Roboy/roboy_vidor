#!/bin/bash
./tools/linux/bossac -i -d --port=/dev/ttyACM$1 -I -U true -i -e -w -v clujtag-server.ino.bin -R
sleep 10
echo "wait..."
./tools/linux/clujtag -p /dev/ttyACM$1 -s boot.svf
BACK_PID=$!
sleep 10
read -r -p "Double-tap reset button quickly, then hit enter" response
if [[ $response =~ ^()$ ]]; then
    ./tools/linux/bossac -i -d --port=/dev/ttyACM$1 -I -U true -i -e -w -v RestoreFPGABootloader.ino.bin -R
fi
echo "done"

