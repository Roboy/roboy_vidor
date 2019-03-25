#!/bin/bash
sudo ./tools/linux/bossac -i -d --port=/dev/ttyACM$1 -I -U true -i -e -w -v clujtag-server.ino.bin -R
sleep 5
echo "wait..."
sudo ./tools/linux/clujtag -p /dev/ttyACM0 -s boot.svf
BACK_PID=$!
sleep 10
read -r -p "Double-tap reset button quickly, then hit enter" response
if [[ $response =~ ^()$ ]]; then
    sudo ./tools/linux/bossac -i -d --port=/dev/ttyACM$1 -I -U true -i -e -w -v RestoreFPGABootloader.ino.bin -R
fi
echo "done"

