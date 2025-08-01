echo -n "Putting device in dfu mode..."
particle usb dfu
echo -n "Flashing via usb"

particle flash --usb ../build/main.bin
