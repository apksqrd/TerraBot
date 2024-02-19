#!/bin/bash

echo "Installing Arduino"
cd Desktop; ln -s /home/robotanist/Desktop/Terrabot .
cd $HOME; ln -s Desktop/TerraBot .
mkdir ~/Sketchbook/libraries; cd ~/Sketchbook/libraries
rosrun rosserial_arduino make_libraries.py .
git clone <https://github.com/RobTillaart/DHT20.git>
sudo usermod -a -G dialout,robotanist $USER
sudo usermod -a -G dialout robotanist
cd ~/TerraBot/lib/ArduinoCode
make clean; make upload

echo "Installing ortools"
sudo apt install swig
cd /usr/lib/python3.8
sudo git <https://github.com/google/or-tools.git>
cd or-tools
sudo make third_party > $HOME/make_third_party.txt
sudo make python
sudo make test
cd ortools; sudo cp init.p*/usr/lib/python3.8; sudo cp init.p* /usr/lib/python3.8/or-tools

echo "-part 2 finished-"
