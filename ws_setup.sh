#!/bin/sh

# installing catkin tools
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update -y

sudo apt-get install python3-catkin-tools -y

# initializing catkins workspace

catkin_make

# sourcing setup.bash and adding it to bashrc
echo "\nsource $(pwd -P)/devel/setup.bash" >>  ~/.bashrc