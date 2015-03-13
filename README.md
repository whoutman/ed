ED : Environment Descriptor [![Build Status](https://travis-ci.org/tue-robotics/ed.svg?branch=master)](https://travis-ci.org/tue-robotics/ed)
======

## Introduction

ED is a 3D geometric, object-based world representation for robots.

## Installation

**Note: Currently only ROS Hydro is supported ** 

We assume you have successfully installed ROS and set-up a Catkin workspace. Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src

    git clone https://github.com/tue-robotics/ed.git
    git clone https://github.com/tue-robotics/tue_filesystem
    git clone https://github.com/tue-robotics/geolib2
    git clone https://github.com/tue-robotics/tue_config.git
    git clone https://github.com/tue-robotics/code_profiler.git
    
You will also need the following system dependencies:

    sudo apt-get install libyaml-dev ros-hydro-std-srvs ros-<DISTRO>-cv-bridge ros-<DISTRO>-tf libassimp-dev ros-<DISTRO>-class-loader 
    
This should be sufficient to successfully compile ED:

    cd <your_catkin_workspace>
    catkin_make
    
## Quick Start

ED strongly relies on plugins to integrate sensor data, estimate object positions, recognize objects, etc. Before you can start ED, you have to specify the location of these plugins using the environment variable ED_PLUGIN_PATH. For example, for a ROS Catkin workspace, the variable should be set to something like:

    export ED_PLUGIN_PATH=<your_catkin_workspace>/devel/lib
    
You can provide multiple paths by seperating them using ':'.

You can then start ED by running:

    rosrun ed ed [config_file]
    
You can specify a configuration file for ED as an optional command line argument. Some example configuration files are in examples.config
