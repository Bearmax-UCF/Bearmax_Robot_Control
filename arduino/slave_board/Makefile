BOARD = arduino:avr:uno

build:
	mkdir -p ./build
	arduino-cli compile -e --build-path ./build -b $(BOARD) .
clean:
	rm -rf ./build &>/dev/null
