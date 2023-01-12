#!/bin/bash

sudo apt-get update && rosdep install -i --from-path src --rosdistro humble -y
