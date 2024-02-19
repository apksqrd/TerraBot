#!/bin/bash

echo "Setup python libraries"


echo "-part 1 finished-"
cd /home/robotanist-admin/.local/lib/
sudo cp -r python3.8/site-packages/*  /usr/lib/python3/dist-packages/.
sudo rm -rf python3.8
