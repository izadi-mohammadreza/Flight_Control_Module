**Install PX4**

Clone the [PX4 Firmware repository](https://github.com/PX4/PX4-Autopilot)

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive

Run the PX4 setup script to install the complete toolchain:

    cd PX4-Autopilot
    bash ./Tools/setup/ubuntu.sh

To make using PX4 easier, you can add the following lines to your .bashrc file:

    export PATH=$PATH:$HOME/PX4-Autopilot/Tools

Don't forget to source your .bashrc after editing:

    source ~/.bashrc

**ROS2 Installation**

Follow this [link](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install ROS2 Foxy or this [link](https://docs.ros.org/en/humble/) to install ROS2 Humble 

**Install Python dependencies**

    pip install --user -U empy pyros-genmsg setuptools
    sudo apt update
    sudo apt install python3-pip
    sudo pip3 install --upgrade pip

**Setup [Micro XRCE-DDS Agent & Client](https://docs.px4.io/main/en/middleware/uxrce_dds.html)**

    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git    
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/



**Install QGroundControl**

QGroundControl can be installed/run on Ubuntu LTS 20.04 (and later).

Follow this link to install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)


**ROS 2 setup**

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/izadi-mohammadreza/Flight_Control_Module
    git clone https://github.com/PX4/px4_msgs.git
    cd ~/ros2_ws
    colcon build --symlink-install

**Disable lockstep in PX4**

    cd ~/PX4-Autopilot
    make px4_sitl boardconfig

Navigate to "Toolchain" and enable "force disable lockstep"

Quit (Q) and save (Y)

Open the gazebo model you will be using for simulation: PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.

Change enable_lockstep (line 466) from 1 to 0 like so:

    <enable_lockstep>0</enable_lockstep>

**Run the Simulation**

1- build PX4 for simulation, use:

    cd ~/PX4-Autopilot
    make px4_sitl_default gazebo 


2- Start the agent

    MicroXRCEAgent udp4 -p 8888

3- run python code

    cd ~/ros2_ws/src/Flight_Control_Module/scripts
    python3 controller_node.py

4- Open the QGroundControl software, arm the drone, and switch to Offboard mode.

5- run python code

    cd ~/ros2_ws/src/Flight_Control_Module/scripts
    python3 circle_trajectory.py



