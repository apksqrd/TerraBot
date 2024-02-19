# Setup Instructions

I'd recommend the RPi specific OS,
especially if you are using a RPi 3.

Set up logins.

Make sure you are connected to the wifi.
You can try to open google.com to make sure.

Set up swap file.

Then run the setup script: `chmod +x rpisetuppart1.sh ; ./rpisetuppart1.sh`.
If you run into errors,
good luck.
I am logging the steps so you should know where something fails.
For a command that fails, you can try again with `sudo` but most commands already have sudo so...

Then, add the line to the end of the .bashrc file: `source /opt/ros/noetic/setup.bash`

If you have terrabot on your computer,  transfer it through USB.
Put it into your rpi's desktop.
(I might even recommend doing this step earlier since you need the script anyways)

Then, run the script: `chmod +X rpisetuppart2.sh ; ./rpisetuppart2.sh`

Add the line to .bashrc (both for robotanist and robotanist-admin): `export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3.8:/usr/lib/python3.8/or-tools`

Finally, run the script: `chmod +X rpisetuppart3.sh ; ./rpisetuppart3.sh`
