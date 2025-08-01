# Check if the build directory exists
if [ ! -d "../build" ]; then
	# If it doesn't exist, create the build directory one level up
	mkdir ../build
fi
particle compile boron .. --saveTo ../build/main.bin
