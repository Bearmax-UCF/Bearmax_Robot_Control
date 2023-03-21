BOARD = arduino:sam:arduino_due_x

build:
	mkdir -p ./build
	arduino-cli compile -e --build-path ./build -b $(BOARD) .
clean:
	rm -rf ./build &>/dev/null
