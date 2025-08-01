read -p "Enter desired device name/id: " name

particle flash ${name} ../build/main.bin
