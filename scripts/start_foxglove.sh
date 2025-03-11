#!/bin/bash
cd src/client/foxglove
yarn web:build:dev
yarn web:serve & sleep 5 
xdg-open http://localhost:8080/  
