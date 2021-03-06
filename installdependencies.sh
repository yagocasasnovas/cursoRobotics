add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

apt-get install ros-lunar-desktop-full --assume-yes

apt-get install ros-lunar-rqt --assume-yes

apt-get install ros-lunar-rqt-common-plugins --assume-yes

apt-get install ros-kinetic-urdfdom.py

rosdep init

rosdep update

echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc

source ~/.bashrc

apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential --assume-yes
