Install PX4

Clone the PX4 Firmware repository

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive

Run the PX4 setup script to install the complete toolchain:
    cd PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh

You can build PX4 for different targets. To build PX4 for simulation, use:
    make px4_sitl_default gazebo 

Or

    make px4_sitl gazebo-classic

To make using PX4 easier, you can add the following lines to your .bashrc file:

    export PATH=$PATH:$HOME/PX4-Autopilot/Tools

Don't forget to source your .bashrc after editing:

    source ~/.bashrc


ROS2 Installation

Follow this link to install ROS2 Foxy or this link to install ROS2 Humble 

Install Python dependencies

    pip install --user -U empy pyros-genmsg setuptools
    sudo apt update
    sudo apt install python3-pip
    sudo pip3 install --upgrade pip

Setup Micro XRCE-DDS Agent & Client

    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git    ( it can be cloned into your ros workspace )
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/

Start the agent
    MicroXRCEAgent udp4 -p 8888

Start the simulation
    cd ~/PX4-Autopilot
    make px4_sitl_default gazebo 
Or
    make px4_sitl gazebo-classic

Install QGroundControl

QGroundControl can be installed/run on Ubuntu LTS 20.04 (and later).

Follow this link to install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

