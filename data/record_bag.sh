#! /usr/bin/bash

cfg_script="$1"

env_name=${cfg_script##*/}
env_name=${env_name%.sh}

timestamp=$(date +"%Y_%m_%d-%H_%M_%S")

bag_name="bags/bag_${env_name}_${timestamp}"

topics=(
	/observations
	/tf
	/tf_static
	/rosout
)

ros2 bag record \
	-o "${bag_name}" \
	--topics "${topics[@]}" \
	--use-sim-time \
	-s mcap --storage-preset-profile zstd_fast
