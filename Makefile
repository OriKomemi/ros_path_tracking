SHELL := /bin/bash

.PHONY: build launch reset setup-venv

all: build reset launch

clean:
	rm -rf build/

setup-venv:
	./setup_venv.sh

build:
	colcon build --symlink-install 

reset:
	./reset_to_start.sh

launch:
	./launch_with_venv.sh
	