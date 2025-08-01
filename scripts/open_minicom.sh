#!/bin/bash

read -p "ttyACM- what? : " port

echo "Opening ttyACM$port"

minicom -b 115200 -o -D /dev/ttyACM"${port}"

