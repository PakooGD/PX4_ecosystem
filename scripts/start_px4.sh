#!/bin/bash

# Чтение параметров PX4 из launch_params.yml
SITL=$(yq e '.px4.sitl' $PROJECT_ROOT/config/launch_params.yml)
MODEL=$(yq e '.px4.model' $PROJECT_ROOT/config/launch_params.yml)

if [ "$SITL" = "true" ]; then
  cd $PROJECT_ROOT/src/client/PX4-Autopilot
  make px4_sitl $MODEL
else
  echo "PX4 SITL отключен в launch_params.yml"
fi
make px4_sitl gz_x500
