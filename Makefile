SHELL := /bin/bash

.PHONY: build launch reset setup-venv

all: build  reset launch

clean:
	rm -rf build/

setup-venv:
	./setup_venv.sh

build:
	colcon build --symlink-install 

source:
	source ~/dev/BGR_Simulator/install/setup.bash
	source install/setup.bash

reset:
	./reset_to_start.sh

launch:
	./launch.sh
	