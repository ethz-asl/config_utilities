# config_utilities
Tools to make working with config structs for (ROS) C++ libraries more uniform, readable, and convenient.

* **Author:** Lukas Schmid <schmluk@mavt.ethz.ch>
* **Affiliation:** Autonomous Systems Lab (ASL), ETH Zürich
* **License:** BSD-3-Clause.

### Table of contents
* [**Why config_utilities**](#Why-config_utilities)
* [**Installation**](#Installation)
* [**Interfaces and Tools** (The config_utilities How To...)](#Interfaces-and-Tools)
* [**Demos**](#Demos)
  * [config_checker](#Config-Checker)
  * [config](#Config)
  * [ros_param](#ROS-Params)
  * [inheritance](#Inheritance)
  
# Why config_utilities
This library was developed to make working with config structs for object-oriented C++ libraries as simple as possible.
Using config_utilities-based configs has the following advantages:

* Having all parameters in a config struct, rather than with other variables, makes code clearer more readable:
  ```c++
  if (x_ < config_.x_max) { doMagic(); }
  ```
* Configs can be easily checked for validity with verbose warnings to avoid runtime issues:
  ```c++
  MyClass::MyClass(const Config& config) : config_(config.checkValid()) {}
  ```
* For projects consisting of a library and a ROS-package, the configs don't have any ROS dependency and can be used in the library.
  In the ROS-package, configs can be created from NodeHandles without requiring additional code:
  ```c++
  MyConfig c = config_utilities::getConfigFromRos<MyConfig>(nh_private);
  ```
* Verbose and clear printing for debugging or verification can be setup for the entire project:
  ```c++
  config_utilities::GlobalSettings::default_print_width = 80;
  std::cout << config.toString() << std::endl;
  ```
* Everything related to a config is located at its definition/implementation, making all its properties clear and easy to change.
  No need for additional fiels or functions where changes could be overlooked.

# Installation
* **Header-Only**

  This mini-library can be used as a header only library by simply copying `config_utilities.hpp` into your project.
   Requires [glog](https://github.com/google/glog) and [xmlrpc++](http://xmlrpc.com/).
  * Dependencies:
    ```sh
    # As System Install:
    sudo apt update
    sudo apt install libxmlrpc-c++8-dev
    sudo apt-get install libgoogle-glog-dev
    ```
    ```sh
    # Alternatively, as Catkin Package:
    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/glog_catkin.git
    catkin build glog_catkin
    ```

* **Demos Package**

  To run the demos, the package can be conveniently installed via catkin: 
  ```sh
  cd ~/catkin_ws/src
  git clone https://github.com/Schmluk/config_utilities.git
  
  # If not already done so, install dependencies:
  git clone git@github.com:ethz-asl/glog_catkin.git
  git clone git@github.com:ethz-asl/gflags_catkin.git
  git clone git@github.com:ethz-asl/minkindr.git
  git clone git@github.com:ethz-asl/eigen_catkin.git
  
  cd config_utilities_demos
  catkin build --this
  ```
  
# Interfaces and Tools
Briefly describes the interfaces available and how to use them.

#### Settings
Set default settings for the entire project. Set these before instantiation of a config.
```c++
config_utilities::GlobalSettings::default_print_width = 80;
config_utilities::GlobalSettings::default_print_indent = 30;
...
```
#### Configs
Define configs by inheriting from the provided config and templating itself. 
All following interfaces are part of `config_utilities::Config`.
```c++
struct MyConfig : public config_utilities::Config<MyConfig> {
  double x_max = 1.0;
};
```
#### Public Member Functions
Use these to interact with a config.
```c++
bool isValid(bool print_warnings=false) const;  // Validity information.
ConfigT checkValid() const;  // Enforce validity.
ConfigT& checkValid(); 
string toString() const;  // Printing.
```

#### Virtual Member Functions
Override these functions to implement the corresponding behavior.
```c++
  virtual void initializeDependentVariableDefaults();  // Initialization.
  virtual void checkParams() const;  // Param validity checks.
  virtual void printFields() const;  // Printing behavior.
  virtual void fromRosParam();  // ROS-creation behavior.
```

#### Protected Member Functions
Use these tools within the virtual functions to create the desired behavior.
```c++
// General settings.
void setConfigName(const std::string& name);
void setPrintWidth(int width);
void setPrintIndent(int indent);

// Set these values in the constructor.
MyConfig::MyConfig() {
  setConfigName("MyConfig");
  ...
}
```
```c++
// Parameter validity constraints.
void checkParamGT<T>(const T& param, const T& value, const std::string& name) const;
void checkParamGE<T>(const T& param, const T& value, const std::string& name) const;
void checkParamLT<T>(const T& param, const T& value, const std::string& name) const;
void checkParamLE<T>(const T& param, const T& value, const std::string& name) const;
void checkParamEq<T>(const T& param, const T& value, const std::string& name) const;
void checkParamNE<T>(const T& param, const T& value, const std::string& name) const;
void checkParamCond(bool condition, const std::string &warning) const;
void checkParamConfig(const Config& config) const;

// Use these checks within checkParams().
MyConfig::checkParams() const {
  checkParamGT(x_max, 0.0, "x_max");
  ...
}
```
```c++
// Printing.
void printField<T>(const std::string& name, const T& field) const;
void printText(const std::string& text) const;

// Use these checks within printFields().
MyConfig::printFields() const {
  printField("x_max", x_max);
  ...
}
```
```c++
// Creation from ROS params.
void rosParam<T>(const std::string& name, T* param);
// Also works for configs, these don't require a name but an optional sub_namespace.
void rosParam(Config* config, const std::string& sub_namespace = "");

// Use these tools within fromRosParam(). Defaults should be set at variable declaration.
MyConfig::fromRosParam() {
  rosParam("x_max", &x_max);
  ...
}
```
 
# Demos
Verbose examples of the most important functionalities are given in the config_utilities_demos package.

## Config Checker
This demo describes how to use the `ConfigChecker` class to verify non-config_utilities configs in a readable way:
```sh
rosrun config_utilities_demos demo_config_checker
```
Runs a validity check and prints all warnings to console:
```
============================== IndependentConfig ===============================
Warning: Param 'a' is expected >= '0' (is: '-1').
Warning: Param 'c' is expected to be 'this is c' (is: 'test').
================================================================================
``` 

## Config
This demo describes how to define custom classes that utilize a `Config` struct:
```sh
rosrun config_utilities_demos demo_config
```
This will setup a class using a valid config and print it to console, as well as a creation attempt with an invalid config:
```
============ MyClass-Config ============
a:             1
b:             2.34
b_half:        1.17
c:             this is c
An_extremely_unecessarily_and_unreasonab
ly_long_param_name: 
               A_similarly_unreasonably_
               long_param_value.
And a custom message.
========================================

============ MyClass-Config ============
Warning: Param 'a' is expected >= '0' (i
         s: '-1').
Warning: Param 'c' is expected to be 'th
         is is c' (is: 'test').
Warning: b is expected > a.
========================================
```

## ROS Params
This demo describes how to use the `config_utilities::getConfigFromRos<Config>()` function to setup configs via the ROS parameter server:
```sh
roscore & rosrun config_utilities_demos demo_ros_params
```
Sets config params from ros and prints them to console:
```
================= Config (from ROS params) =================
a:        123
b:        45.6
c:        seven-eight-nine
vec:      [1, 2, 3]
map:      {m1: 1, m2: 2}
T:        t: [0, 0, 0] RPY°: [-0, 0, -0]
============================================================
``` 

## Inheritance
This demo describes how to use nested configs, which can be used to setup derived and base classes:
```sh
roscore & rosrun config_utilities_demos demo_inheritance
```
Sets up a derived class from ROS, prints its nested config, and check for validity:
```
===================== MyDerivedConfig ======================
e:                  Bananas are yellow.
f:                  6
other_config:
   a:               -11.1
   b:               222
base_config:
   c:               False
   d:               3.45
   other_config:
      a:            1
      b:            2
============================================================

======================= OtherConfig ========================
Warning: Param 'a' is expected >= '0' (is: '-1').
============================================================
========================== MyBase ==========================
Warning: Member config 'OtherConfig' is not valid.
============================================================
===================== MyDerivedConfig ======================
Warning: Member config 'MyBase' is not valid.
============================================================

``` 
