set history save
target extended-remote /dev/cu.usbmodemE3C3DCC51
set mem inaccessible-by-default off
monitor swdp_scan
file .pio/build/default/firmware.elf

