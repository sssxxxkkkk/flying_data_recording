#!/bin/bash

source /media/songxiaokai/E1/svn/kalibr2/devel/setup.bash
#rosrun kalibr kalibr_calibrate_imu_camera --target checkerboard.yaml --bag /media/songxiaokai/F/dataset/guafei_calib/output.bag --cam output-camchain.yaml --imu imu.yaml --show-extraction 
rosrun kalibr kalibr_calibrate_imu_camera --target checkerboard.yaml --bag ../../save_data/output.bag --cam output-camchain.yaml --imu imu.yaml --show-extraction --timeoffset-padding 0.2
