#!/bin/bash

source ../../../kalibr2/devel/setup.bash


rosrun kalibr kalibr_calibrate_cameras --target checkerboard.yaml --bag ../../save_data/output.bag --models pinhole-radtan --topics /camera/image_raw --show-extraction
