#!/bin/bash
read -r -p "Double-tap reset button quickly, then hit enter" response
./tools/linux/bossac -i -d --port=/dev/ttyACM0 -I -U true -i -e -w -v clujtag-server.ino.bin -R
echo "wait..."
./tools/linux/clujtag -p /dev/ttyACM0 -s boot.svf
read -r -p "Double-tap reset button quickly, then hit enter" response
./tools/linux/bossac -i -d --port=/dev/ttyACM0 -I -U true -i -e -w -v RestoreFPGABootloader.ino.bin -R
echo "done"
