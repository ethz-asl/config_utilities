# config_utilities
Tools to make working with config structs for (ROS) C++ libraries more uniform, readable, and convenient.

* **Author:** Lukas Schmid <schmluk@mavt.ethz.ch>
* **License:** BSD-3-Clause.

### Table of contents
* [Installation](#Installation)
* [Demo](#Demo)
* [Tools](#Tools)
  * [Config](#Config)
  * [Config Checker](#Config-Checker)
  

## Installation
* **Header-Only**

  This mini-library can be used as a header only library by simply copying the `include/config.h` into your project.
  Requires a system install of google glog.
  
  * Dependencies:
    ```
    sudo apt update
    sudo apt install libxmlrpc-c++8-dev
    sudo apt-get install libgoogle-glog-dev
    ```

* **Catkin Package**

  To use within a ROS workspace it can be conveniently installed via catkin. 
  ```sh
  cd ~/catkin_ws/src
  git clone https://github.com/Schmluk/config_utilities.git
  
  # If not already done so install google logging
  git clone git@github.com:ethz-asl/glog_catkin.git
  git clone git@github.com:ethz-asl/gflags_catkin.git
  
  cd config_utilities
  catkin build --this
  ```
## Demo
A verbose example of the most important features is given in `src/demo.cpp`. 
To run it type:
```sh
rosrun config_utilities_demos demo_config --logtostderr
```
It shows how to setup a class using configs and print it to console:
```
============ MyClass-Config ============
a:                  0
b:                  123.4
b_half:             61.7
c:                  this is c
And a custom message.
========================================
``` 
And how invalid configs are handled:
```
W0803 config.h:71] MyClass-Config: param 'a' is expected >= '0' (is: '-1').
W0803 config.h:98] MyClass-Config: param 'c' is expected to be 'this is c' (is: 'test').
W0803 config.h:106] MyClass-Config: b is expected > a.
F0803 config.h:155] Check failed: result.isValid() 

```


## Tools
### Config

### Config Checker
