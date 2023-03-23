COMPILER=colcon
OPTIONS=--symlink-install
COMPILE=$(COMPILER) build $(OPTIONS)
BUILD=build
INSTALL=install
LOGS=logs

all:
	$(COMPILE)

clean:
	echo "$(BUILD) $(INSTALL) $(LOGS)" | xargs -I _ rm -rf _ >/dev/null

.PHONY: clean
