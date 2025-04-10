#!/bin/bash

# Чтение параметров PX4 из launch_params.yml
MODEL=$(yq e '.px4.model' $PROJECT_ROOT/config/launch_params.yml)

cd $PROJECT_ROOT/src/client/PX4-Autopilot
make px4_sitl $MODEL 
