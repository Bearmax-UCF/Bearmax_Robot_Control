BOARD = arduino:avr:mega

build:
	mkdir -p ./build
	arduino-cli compile -e --build-path ./build -b $(BOARD) .
clean:
	rm -rf ./build &>/dev/null
